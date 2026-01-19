package org.steelhawks.subsystems.shooter;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.util.LoggedTunableNumber;

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


    }
}

