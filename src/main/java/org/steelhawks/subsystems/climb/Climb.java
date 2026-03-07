package org.steelhawks.subsystems.climb;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

public class Climb extends SubsystemBase {
    private final Debouncer hominhDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final ClimbIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;
    private Rotation2d desiredRotation = new Rotation2d();
    private ClimbConstants.ClimbState desiredState = ClimbConstants.ClimbState.RETRACTED;
    private boolean brakeModeEnabled = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @AutoLogOutput(key = "Climb/PositionRad")
    private double getPosition() {
        return inputs.positionRad;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    @AutoLogOutput(key = "Climb/AtGoal")
    public boolean atGoal() {
        return atGoal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        if (!isHomed && Toggles.Climb.isEnabled.get()) {
            io.runPercentOut(ClimbConstants.HOMING_VOLTS);
            isHomed = hominhDebouncer.calculate(inputs.supplyCurrentAmps > 40);
            Logger.recordOutput("Climb/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(0);
                io.stop();
                isZeroed = true;
                desiredRotation = new Rotation2d();
                Logger.recordOutput("Climb/Zeroed", true);
            }
        }

        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }

        final boolean shouldRun =
            DriverStation.isEnabled()
                && (isHomed && isZeroed)
                && inputs.connected
                && Toggles.Climb.isEnabled.get()
                && !Toggles.Climb.toggleVoltageOverride.get()
                && !Toggles.Climb.toggleCurrentOverride.get()
                && (getPosition() <= ClimbConstants.MAX_POSITION.getRadians()
                && getPosition() >= ClimbConstants.MIN_POSITION.getRadians());

        Logger.recordOutput("Climb/ShouldRun", shouldRun);

        if (Toggles.tuningMode.get()) {
            if (Toggles.Climb.toggleVoltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Hood/TuningVolts");
                }
                io.runOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Climb.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Hood/TuningAmps");
                }
                io.runOpenLoop(tuningAmps.get(), true);
            }

            LoggedTunableNumber.ifChanged(
                this.hashCode(),
                () -> {
                    io.setPIDFF(
                        ClimbConstants.kP.get(),
                        ClimbConstants.kI.get(),
                        ClimbConstants.kD.get(),
                        ClimbConstants.kV.get(),
                        ClimbConstants.kA.get(),
                        ClimbConstants.MOTIONMAGIC_EXPO_CRUISE_VELOCITY.get()
                    );
                }, ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD, ClimbConstants.kV, ClimbConstants.kA, ClimbConstants.MOTIONMAGIC_EXPO_CRUISE_VELOCITY
            );

        }

        atGoal = Maths.epsilonEquals(getPosition(), desiredRotation.getRadians(), ClimbConstants.TOLERANCE);
        if (shouldRun) {
            if (atGoal) {
                io.stop();
            } else {
                io.runPosition(
                    desiredRotation,
                    ClimbConstants.kS.get() + ClimbConstants.kG.get()
                );
            }
        }

        Logger.recordOutput("Climb/DesiredState", desiredState);
    }

    public void setDesiredState(ClimbConstants.ClimbState state) {
        desiredRotation = Rotation2d.fromRadians(
            MathUtil.clamp(state.position, ClimbConstants.MIN_POSITION.getRadians(), ClimbConstants.MAX_POSITION.getRadians()
        ));
        desiredState = state;
    }

    public void setDesiredPosition(double radians) {
        desiredRotation = Rotation2d.fromRadians(
            MathUtil.clamp(radians, ClimbConstants.MIN_POSITION.getRadians(), ClimbConstants.MAX_POSITION.getRadians())
        );
    }

    public Command setDesiredStateCommand(ClimbConstants.ClimbState state) {
        return Commands.runOnce(() -> setDesiredState(state), this);
    }

    public Command setDesiredPositionCommand(double radians) {
        return Commands.runOnce(() -> setDesiredPosition(radians), this);
    }

    public ClimbConstants.ClimbState getDesiredState() {
        return desiredState;
    }
}

