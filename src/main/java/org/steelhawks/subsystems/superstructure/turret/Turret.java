package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.LoopTimeUtil;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA");
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");

    private static final LoggedTunableNumber maxVelocityRadPerSec = new LoggedTunableNumber("Turret/MaxVelocityRadPerSec");
    private static final LoggedTunableNumber maxAccelerationRadPerSecSq = new LoggedTunableNumber("Turret/MaxAccelerationRadPerSecSq");
    private static final LoggedTunableNumber tolerance = new LoggedTunableNumber("Turret/Tolerance", 0.1);
    private static final LoggedTunableNumber manualIncrement = new LoggedTunableNumber("Turret/ManualIncrement", 0.3);

    private static final Rotation2d minRotation = new Rotation2d();
    private static final Rotation2d maxRotation = new Rotation2d();
    public static int motorId = 1;

    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private TrapezoidProfile profile;
    private final TurretIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private Rotation2d desiredRotation = new Rotation2d();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private boolean brakeModeEnabled = false;
    private boolean isManual = false;
    private boolean atGoal = false;
    private DoubleSupplier joystickAxis = null;

    public Turret(TurretIO io) {
        this.io = io;
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    maxVelocityRadPerSec.getAsDouble(),
                    maxAccelerationRadPerSecSq.getAsDouble()));
    }

    private Rotation2d getPosition() {
        return inputs.positionRad;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public boolean atGoal() {
        return atGoal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        final boolean shouldRun =
            DriverStation.isEnabled()
                && !isManual
                && inputs.connected
                && Toggles.Turret.isEnabled.get()
                && !Toggles.Turret.toggleVoltageOverride.get()
                && !Toggles.Turret.toggleCurrentOverride.get()
                && (getPosition().getRadians() <= maxRotation.getRadians()
                    && getPosition().getRadians() >= minRotation.getRadians());

        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(getPosition().getRadians(), 0.0);
        }
        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }
        if (Toggles.tuningMode.get()) {
            if (Toggles.Turret.toggleVoltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Turret/TuningVolts", 0.0);
                }
                io.runOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Turret.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Turret/TuningAmps", 0.0);
                }
                io.runOpenLoop(tuningAmps.get(), true);
            }
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setPID(
                    kP.get(),
                    kI.get(),
                    kD.get());
            }, kP, kI, kD);
            if (maxVelocityRadPerSec.hasChanged(hashCode())
                || maxAccelerationRadPerSecSq.hasChanged(hashCode())
            ) {
                profile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            maxVelocityRadPerSec.get(),
                            maxAccelerationRadPerSecSq.get()));
            }
        }
        if (shouldRun) {
            desiredRotation =
                Rotation2d.fromRadians(
                    MathUtil.clamp(
                        desiredRotation.getRadians(), minRotation.getRadians(), maxRotation.getRadians()));
            goal = new TrapezoidProfile.State(desiredRotation.getRadians(), 0.0);
            double previousVelocity = setpoint.velocity;
            setpoint =
                profile
                    .calculate(Constants.UPDATE_LOOP_DT, setpoint, goal);
            if (setpoint.position < minRotation.getRadians()
                || setpoint.position > maxRotation.getRadians()
            ) {
                setpoint =
                    new TrapezoidProfile.State(
                        MathUtil.clamp(setpoint.position, minRotation.getRadians(), maxRotation.getRadians()),
                        0.0);
            }
            atGoal = Math.abs(getPosition().getRadians() - goal.position) <= tolerance.getAsDouble();
            if (atGoal) {
                io.stop();
            } else {
                double acceleration = (setpoint.velocity - previousVelocity) / Constants.UPDATE_LOOP_DT;
                io.runPivot(
                    setpoint.position,
                    kS.getAsDouble() * Math.signum(setpoint.velocity)
                        + kA.getAsDouble() * acceleration
                );
            }
            Logger.recordOutput("Turret/SetpointPosition", setpoint.position);
            Logger.recordOutput("Turret/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Turret/GoalPosition", goal.position);
            Logger.recordOutput("Turret/GoalVelocity", goal.velocity);
        } else {
            setpoint = new TrapezoidProfile.State(getPosition().getRadians(), 0.0);
            Logger.recordOutput("Turret/SetpointPosition", 0.0);
            Logger.recordOutput("Turret/SetpointVelocity", 0.0);
            Logger.recordOutput("Turret/GoalPosition", 0.0);
            Logger.recordOutput("Turret/GoalVelocity", 0.0);
        }
        if (isManual) {
            if (joystickAxis != null) {
                double appliedSpeed =
                    joystickAxis.getAsDouble() * manualIncrement.getAsDouble();
                boolean canMoveCCW = appliedSpeed > 0 && getPosition().getRadians() < maxRotation.getRadians();
                boolean canMoveCW = appliedSpeed < 0 && getPosition().getRadians() > minRotation.getRadians();
                if (canMoveCCW || canMoveCW) {
                    io.runPercentOutput(appliedSpeed);
                }
            }
        }
        LoopTimeUtil.record("Turret");
    }

    public Command setDesiredState(Rotation2d state) {
        return Commands.runOnce(
                () -> {
                    desiredRotation =
                        Rotation2d.fromRadians(
                            MathUtil.clamp(
                                state.getRadians(), minRotation.getRadians(), maxRotation.getRadians()));
                }, this)
            .withName("Set Desired State");
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.runOnce(() -> {
            isManual = true;
            Logger.recordOutput("Turret/RequestedSpeed", joystickAxis.getAsDouble());
            this.joystickAxis = joystickAxis;
        }, this);
    }
}
