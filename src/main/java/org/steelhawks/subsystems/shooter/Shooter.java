package org.steelhawks.subsystems.shooter;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;
import java.lang.Math;
import java.sql.Driver;


public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIO.ShooterIOInputs inputs;
    private final LoggedTunableNumber tuningVolts;

    public double getVelocity() {
        return inputs.masterMotorVelocityRPM;
    }

    //    private final TalonFX shooterMotor =
//        new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID_OMEGA, ShooterConstants.CAN_NAME);
//
//    private final VelocityVoltage voltageSetting =
//        new VelocityVoltage(0.0);

    public Shooter(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterIO.ShooterIOInputs();
    }

    public double getVelocityValue() {
        return inputs.masterMotorVelocityRPM;
    }

    public double getDesiredVelocityValue(double distance, double shooterHeight) {
        return 0; //physics
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            io.setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            io.setBrakeMode(true);
        }
        if (Toggles.tuningMode.get()) {
            if (Toggles.Shooter.toggleVoltageOverride.getAsBoolean()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Shooter/TuningVolts", 0);
                }
                io.runOpenLoop(tuningVolts.get());
            }
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setPID(
                    ShooterConstants.kP.get(),
                    ShooterConstants.kI.get(),
                    ShooterConstants.kD.get()
                );
            }, ShooterConstants.kP);
        }
        boolean shouldRun =
            DriverStation.isEnabled() &&
                !Toggles.Shooter.toggleVoltageOverride.getAsBoolean();

        Logger.recordOutput("Shooter/Running", shouldRun);
        inputs.shouldRunProfile = shouldRun;

        if (shouldRun) {
            inputs.goal =
                MathUtil.clamp(getDesiredVelocityValue(500), //find distance w/ vision
                    0,
                    ShooterConstants.MAX_VELOCITY_METERPERSEC);
        }
    }
}