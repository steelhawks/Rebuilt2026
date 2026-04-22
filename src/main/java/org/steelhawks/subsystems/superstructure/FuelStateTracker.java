package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotState;
import org.steelhawks.subsystems.beam.BeamIOInputsAutoLogged;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.Supplier;

public class FuelStateTracker {
    private FuelState currentGamePieceState = FuelState.NONE;
    private FuelState previousGamePieceState;

    private double lastTransitionTime = Timer.getFPGATimestamp();
    private final double TIMEOUT_SECONDS = 3.0;


    private Supplier<RobotState.AimState> aimStateSupplier;
    private Supplier<Double> flywheelVelocitySupplier;
    private Supplier<Boolean> shooterAtSpeedSupplier;
    private Supplier<Boolean> intakeRunningSupplier;
    private Supplier<Boolean> hopperSupplier;

    private final LoggedTunableNumber velocityDropThreshold = new LoggedTunableNumber("Velocity Drop Threshold", 200);



    public void updateAtIntake(boolean value) {
        intakeRunningSupplier = () -> value;
        update();
    }

    public void updateAtSpindexer(boolean value) {
        hopperSupplier = () -> value;
        update();
    }

    public void updateWhenFerrying(RobotState.AimState value) {
        aimStateSupplier = () -> value;
        update();
    }

    public void updateAtShooter(boolean value, double flywheelVelocity) {
        shooterAtSpeedSupplier = () -> value;
        flywheelVelocitySupplier = () -> flywheelVelocity;
        update();
    }



    public void update() {
        previousGamePieceState = currentGamePieceState;
        boolean beamBroken = hopperSupplier.get();
        boolean isFerry = aimStateSupplier.get() == RobotState.AimState.FERRY;
        boolean shooterReady = shooterAtSpeedSupplier.get();
        boolean intakeRolling = intakeRunningSupplier.get();

        double now =  Timer.getFPGATimestamp();

        currentGamePieceState = switch (currentGamePieceState) {
            case NONE -> FuelState.EMPTY;
            case EMPTY -> {
                if (beamBroken) currentGamePieceState = FuelState.HOLDING; lastTransitionTime = now;
                if (isFerry) currentGamePieceState = FuelState.FERRY; lastTransitionTime = now;
                if (intakeRolling) currentGamePieceState = FuelState.INTAKING; lastTransitionTime = now;
                yield FuelState.EMPTY;
            }
            case INTAKING -> {
                if (beamBroken) currentGamePieceState =  FuelState.HOLDING; lastTransitionTime = now;
                if (isFerry) currentGamePieceState = FuelState.FERRY; lastTransitionTime = now;
                yield FuelState.INTAKING;
            }
            case FERRY -> {
                if (beamBroken) {
                    currentGamePieceState = FuelState.HOLDING;
                    lastTransitionTime = now;
                } else if (!isFerry) {
                    currentGamePieceState = FuelState.EMPTY;
                    lastTransitionTime = now;
                } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
                    currentGamePieceState = FuelState.NONE;
                    lastTransitionTime = now;
                }
                yield FuelState.FERRY;
            }
            case HOLDING -> {
                if (!beamBroken) {
                   currentGamePieceState = FuelState.EMPTY;
                   lastTransitionTime = now;
                } else if (shooterReady) {
                    currentGamePieceState = FuelState.INDEXING;
                    lastTransitionTime = now;
                } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
                    currentGamePieceState = FuelState.NONE;
                    lastTransitionTime = now;
                }
                yield FuelState.HOLDING;
            }
            case INDEXING -> {
                if (!beamBroken) yield FuelState.SHOOTING;
                yield FuelState.INDEXING;
            }
            case SHOOTING -> {
                if (beamBroken) yield FuelState.HOLDING;  // next ball ready
                if (flywheelVelocitySupplier.get() < velocityDropThreshold.get())
                    yield FuelState.EMPTY;  // velocity drop = ball exited, hopper empty
                if (now - lastTransitionTime > TIMEOUT_SECONDS) {
                     currentGamePieceState =  FuelState.NONE;
                     lastTransitionTime = now;
                }
                yield FuelState.SHOOTING;
            }
        };

        Logger.recordOutput("GamePieceTracker/State", currentGamePieceState.toString());
        Logger.recordOutput("GamePieceTracker/HasBalls", hasBalls());
        Logger.recordOutput("GamePieceTracker/BeamBroken", beamBroken);
    }


    public FuelState getFuelPosition() {
        return currentGamePieceState;
    }

    public boolean hasBalls() {
        return currentGamePieceState.hasBalls;
    }

    public void forceSetState(FuelState newState) {
        currentGamePieceState = newState;
        lastTransitionTime = Timer.getFPGATimestamp();
    }

}
