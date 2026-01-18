package org.steelhawks.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.steelhawks.Constants;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    public final TalonFX flywheelMotor1;
    public final TalonFX flywheelMotor2;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<AngularVelocity> motor1Velocity;
    private final StatusSignal<Voltage> motor1Voltage;
    private final StatusSignal<Current> motor1Amperage;
    private final StatusSignal<Temperature> motor1Temp;

    private final StatusSignal<AngularVelocity> motor2Velocity;
    private final StatusSignal<Voltage> motor2Voltage;
    private final StatusSignal<Current> motor2Amperage;
    private final StatusSignal<Temperature> motor2Temp;

    private final VelocityVoltage velocityOut;
    private final VoltageOut voltageOut;

    public ShooterIOTalonFX(RobotConfig.CANBus canBus) {
        flywheelMotor1 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_1_ID, canBus.bus);
        flywheelMotor2 = new TalonFX(ShooterConstants.SHOOTER_MOTOR_2_ID, canBus.bus);
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

        config.Slot0.kP = ShooterConstants.kP.getAsDouble();
        config.Slot0.kI = ShooterConstants.kI.getAsDouble();
        config.Slot0.kD = ShooterConstants.kD.getAsDouble();
        config.Feedback.RotorToSensorRatio = ShooterConstants.GEAR_RATIO;

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor1.getConfigurator().apply(config));

        motor1Velocity = flywheelMotor1.getVelocity();
        motor1Voltage = flywheelMotor1.getMotorVoltage();
        motor1Temp = flywheelMotor1.getDeviceTemp();
        motor1Amperage = flywheelMotor1.getStatorCurrent();

        motor2Velocity = flywheelMotor1.getVelocity();
        motor2Voltage = flywheelMotor1.getMotorVoltage();
        motor2Temp = flywheelMotor1.getDeviceTemp();
        motor2Amperage = flywheelMotor1.getStatorCurrent();

        PhoenixUtil.registerSignals(false,
                motor1Amperage,
                motor1Voltage,
                motor1Temp,
                motor1Velocity,
                motor2Amperage,
                motor2Voltage,
                motor2Temp,
                motor2Velocity
        );

        flywheelMotor1.optimizeBusUtilization();

        velocityOut = new VelocityVoltage(0.0);
        voltageOut = new VoltageOut(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motor1Connected = BaseStatusSignal.isAllGood(motor1Voltage, motor1Velocity, motor1Temp, motor1Amperage);
        inputs.motor1VelocityRadPerSec = motor1Velocity.getValueAsDouble();
        inputs.motor1AppliedVolts = motor1Voltage.getValueAsDouble();
        inputs.motor1Temp = motor1Temp.getValueAsDouble();
        inputs.motor1Current = motor1Amperage.getValueAsDouble();

        inputs.motor2Connected = BaseStatusSignal.isAllGood(motor2Voltage, motor2Velocity, motor2Temp, motor2Amperage);
        inputs.motor2VelocityRadPerSec = motor1Velocity.getValueAsDouble();
        inputs.motor2AppliedVolts = motor1Voltage.getValueAsDouble();
        inputs.motor2Temp = motor1Temp.getValueAsDouble();
        inputs.motor2Current = motor1Amperage.getValueAsDouble();
    }

    @Override
    public void runOpenLoop(double volts) {
        flywheelMotor1.setControl(
                voltageOut.withOutput(volts)
        );
    }

    @Override
    public void runVelocity(double rps, double feedforward) {
        flywheelMotor1.setControl(
                velocityOut.withVelocity(rps)
                        .withFeedForward(feedforward)
        );
    }

    @Override
    public void stop() {
        flywheelMotor1.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor1.getConfigurator().apply(config));
    }
}
