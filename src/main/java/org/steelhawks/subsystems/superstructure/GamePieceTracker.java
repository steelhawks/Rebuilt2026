package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotState;
import org.steelhawks.subsystems.beam.BeamIOInputsAutoLogged;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.Supplier;

public class GamePieceTracker {
    private GamePieceState currentGamePieceState;
    private GamePieceState previousGamePieceState;
    private final BeamIOInputsAutoLogged inputs;

    private final Supplier<RobotState.AimState> aimStateSupplier;
    private final Supplier<Double> flywheelVelocitySupplier;
    private final Supplier<Boolean> shooterAtSpeedSupplier;
    private final Supplier<Boolean> intakeRunningSupplier;

    private final LoggedTunableNumber velocityDropThreshold;

    public GamePieceTracker(
            BeamIOInputsAutoLogged inputs,
            Supplier<RobotState.AimState> aimStateSupplier,
            Supplier<Double> flywheelVelocitySupplier,
            Supplier<Boolean> shooterAtSpeedSupplier,
            Supplier<Boolean> intakeRunningSupplier
    ) {
        velocityDropThreshold = new LoggedTunableNumber("Velocity Drop Threshold", 0.0);
        this.inputs = inputs;
        this.aimStateSupplier = aimStateSupplier;
        this.flywheelVelocitySupplier = flywheelVelocitySupplier;
        this.shooterAtSpeedSupplier = shooterAtSpeedSupplier;
        this.intakeRunningSupplier = intakeRunningSupplier;
    }



    public void update() {
        previousGamePieceState = currentGamePieceState;
        boolean beamBroken = inputs.detected;
        boolean isFerry = aimStateSupplier.get() == RobotState.AimState.FERRY;
        boolean shooterReady = shooterAtSpeedSupplier.get();
        boolean intakeRolling = intakeRunningSupplier.get();

        currentGamePieceState = switch (currentGamePieceState) {
            case EMPTY -> {
                if (beamBroken) yield GamePieceState.HOLDING;
                if (isFerry) yield GamePieceState.FERRY;
                if (intakeRolling) yield GamePieceState.INTAKING;
                yield GamePieceState.EMPTY;
            }
            case INTAKING -> {
                if (beamBroken) yield GamePieceState.HOLDING;
                if (isFerry) yield GamePieceState.FERRY;
                yield GamePieceState.INTAKING;
            }
            case FERRY -> {
                if (beamBroken) yield GamePieceState.HOLDING;
                if (!isFerry) yield GamePieceState.EMPTY;
                yield GamePieceState.FERRY;
            }
            case HOLDING -> {
                if (!beamBroken) yield GamePieceState.EMPTY;
                if (shooterReady) yield GamePieceState.INDEXING;
                yield GamePieceState.HOLDING;
            }
            case INDEXING -> {
                if (!beamBroken) yield GamePieceState.SHOOTING;
                yield GamePieceState.INDEXING;
            }
            case SHOOTING -> {
                if (beamBroken) yield GamePieceState.HOLDING;  // next ball ready
                if (flywheelVelocitySupplier.get() < velocityDropThreshold.get())
                    yield GamePieceState.EMPTY;  // velocity drop = ball exited, hopper empty
                yield GamePieceState.SHOOTING;
            }
        };

        Logger.recordOutput("GamePieceTracker/State", currentGamePieceState.toString());
        Logger.recordOutput("GamePieceTracker/HasBalls", hasBalls());
        Logger.recordOutput("GamePieceTracker/BeamBroken", beamBroken);
    }


    public GamePieceState getCurrentState() {
        return currentGamePieceState;
    }

    public boolean hasBalls() {
        return currentGamePieceState.hasBalls;
    }

    public boolean isEmpty() {
        return currentGamePieceState == GamePieceState.EMPTY;
    }

    public boolean isReadyToShoot() {
        return currentGamePieceState == GamePieceState.INDEXING
                || currentGamePieceState == GamePieceState.SHOOTING;
    }

    public boolean isShooting() {
        return currentGamePieceState == GamePieceState.SHOOTING;
    }

    // Fires once on state entry — useful for triggering auto-stage commands
    public boolean justEnteredState(GamePieceState state) {
        return currentGamePieceState == state && previousGamePieceState != state;
    }

    public Trigger hasBallsTrigger() {
        return new Trigger(this::hasBalls);
    }

    public Trigger isEmptyTrigger() {
        return new Trigger(this::isEmpty);
    }

    public Trigger isReadyToShootTrigger() {
        return new Trigger(this::isReadyToShoot);
    }

    public Trigger justEnteredStateTrigger(GamePieceState state) {
        return new Trigger(() -> justEnteredState(state));
    }

}
