package org.steelhawks.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.FieldConstants;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.superstructure.ShooterConstants;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

public class Hood extends SubsystemBase {

    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private Rotation2d setpoint = ShooterConstants.Hood.MAX_ANGLE;
    private boolean brakeModeEnabled = false;
    private boolean atGoal = false;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        final boolean shouldRun =
            DriverStation.isEnabled()
            && (inputs.motorConnected && inputs.cancoderConnected)
            && Toggles.Hood.isEnabled.get()
            && !Toggles.Hood.voltageOverride.get()
            && !Toggles.Hood.currentOverride.get()
            && (getPositionDeg() >= ShooterConstants.Hood.MIN_ANGLE.getDegrees())
            && (getPositionDeg() <= ShooterConstants.Hood.MAX_ANGLE.getDegrees());
        Logger.recordOutput("Hood/ShouldRun", shouldRun);

        if (DriverStation.isDisabled()) {
            setpoint = Rotation2d.fromDegrees(getPositionDeg());
        }
        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            Toggles.Hood.disableBrakeMode.set(true);
        }
        setBrakeMode(!Toggles.Hood.disableBrakeMode.get());
        if (Toggles.tuningMode.get()) {
            if (Toggles.Hood.voltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Intake/TuningVolts", 0);
                }
                io.runOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Hood.currentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Intake/TuningAmps", 0);
                }
                io.runOpenLoop(tuningAmps.get(), true);
            }
            LoggedTunableNumber.ifChanged(
                this.hashCode(),
                () -> io.setPID(
                    ShooterConstants.Hood.kP.get(),
                    ShooterConstants.Hood.kI.get(),
                    ShooterConstants.Hood.kD.get()
                ), ShooterConstants.Hood.kP, ShooterConstants.Hood.kI, ShooterConstants.Hood.kD
            );
        }
        if (shouldRun) {
            var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
            setDesiredPosition(Rotation2d.fromRadians(ShooterStructure.Static.calculateShot(hubCenter, hubCenter).hoodAngle()));
            atGoal = Maths.epsilonEquals(getPositionDeg(), setpoint.getDegrees(), ShooterConstants.Hood.TOLERANCE);
            io.runHoodPosition(
                setpoint,
                ShooterConstants.Hood.kG.get());
        }
    }

    public boolean atGoal() {
        return atGoal;
    }

    public void setDesiredPosition(Rotation2d position) {
        inputs.goal = MathUtil.clamp(
            position.getDegrees(),
            ShooterConstants.Hood.MIN_ANGLE.getDegrees(),
            ShooterConstants.Hood.MAX_ANGLE.getDegrees());
        setpoint = Rotation2d.fromDegrees(inputs.goal);
    }

    public Command setDesiredPositionCommand(Rotation2d position) {
        return Commands.runOnce(() -> setDesiredPosition(position));
    }

    public double getPositionDeg() {
        return inputs.cancoderPositionDeg.getDegrees();
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }
}
