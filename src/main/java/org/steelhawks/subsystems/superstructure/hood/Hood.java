package org.steelhawks.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

public class Hood extends SubsystemBase {

    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private final Debouncer homingDebouncer =
        new Debouncer(0.25, Debouncer.DebounceType.kRising);

    private Rotation2d setpoint;
    private boolean brakeModeEnabled = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;
    private final double homingVolts = 4.0;
    private final SubsystemConstants.HoodConstants constants;

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kG;

    public Hood(HoodIO io, SubsystemConstants.HoodConstants constants) {
        this.io = io;
        this.constants = constants;
        setpoint = constants.maxAngle();
        kP = new LoggedTunableNumber("Hood/kP", constants.kP());
        kI = new LoggedTunableNumber("Hood/kI", constants.kI());
        kD = new LoggedTunableNumber("Hood/kD", constants.kD());
        kS = new LoggedTunableNumber("Hood/kS", constants.kS());
        kG = new LoggedTunableNumber("Hood/kG", constants.kG());
        inputs.goal = 80.0;
    }

    @AutoLogOutput(key = "Hood/IsStalling")
    private boolean isStalling() {
        return homingDebouncer.calculate(Math.abs(inputs.torqueCurrentAmps) > 50.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (!isHomed && Toggles.Hood.isEnabled.get()) {
            io.runOpenLoop(homingVolts, false);
            isHomed = isStalling();
            Logger.recordOutput("Intake/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(Rotation2d.fromDegrees(80.0));
                io.stop();
                isZeroed = true;
                Logger.recordOutput("Intake/Zeroed", true);
            }
        }

        final boolean shouldRun =
            DriverStation.isEnabled()
                && ((isHomed && isZeroed) || Constants.getRobot().equals(Constants.RobotType.SIMBOT))
                && (inputs.motorConnected && inputs.cancoderConnected)
                && Toggles.Hood.isEnabled.get()
                && !Toggles.Hood.voltageOverride.get()
                && !Toggles.Hood.currentOverride.get()
                && (!isHomed || (getPositionDeg() >= constants.minAngle().getDegrees()
                    && getPositionDeg() <= constants.maxAngle().getDegrees()));
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
                    tuningVolts = new LoggedTunableNumber("Hood/TuningVolts", 0);
                }
                io.runOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Hood.currentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Hood/TuningAmps", 0);
                }
                io.runOpenLoop(tuningAmps.get(), true);
            }
            LoggedTunableNumber.ifChanged(
                this.hashCode(),
                () -> io.setPID(
                    kP.get(),
                    kI.get(),
                    kD.get()
                ), kP, kI, kD
            );
        }
        if (shouldRun) {
//            if (!Toggles.shooterTuningMode.get()) {
//                var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
//                setDesiredPosition(Rotation2d.fromRadians(ShooterStructure.Static.calculateShot(hubCenter, hubCenter).hoodAngle()));
//            }
            atGoal = Maths.epsilonEquals(getPositionDeg(), setpoint.getDegrees(), constants.tolerance());
            io.runHoodPosition(
                setpoint, 0.0);
        }
    }

    @AutoLogOutput(key = "Hood/AtGoal")
    public boolean atGoal() {
        return atGoal;
    }

    public void setDesiredPosition(Rotation2d position) {
        if (Toggles.shooterTuningMode.get()) return;
        inputs.goal = MathUtil.clamp(
            position.getDegrees(),
            constants.minAngle().getDegrees(),
            constants.maxAngle().getDegrees());
        setpoint = Rotation2d.fromDegrees(inputs.goal);
    }

    public void setDesiredPositionForced(Rotation2d position) {
        inputs.goal = MathUtil.clamp(
            position.getDegrees(),
            constants.minAngle().getDegrees(),
            constants.maxAngle().getDegrees());
        setpoint = Rotation2d.fromDegrees(inputs.goal);
    }

    public Command zeroHood() {
        return Commands.runOnce(() -> {
            isHomed = false;
            isZeroed = false;
            homingDebouncer.calculate(false);
        }, this);
    }

    public Command setDesiredPositionCommand(Rotation2d position) {
        return Commands.runOnce(() -> setDesiredPosition(position));
    }

    public double getPositionDeg() {
        return inputs.motorPositionDeg.getDegrees();
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }
}
