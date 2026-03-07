package org.steelhawks.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.Serializers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;

public class ClimbIOTalonFX implements ClimbIO {
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temperature;

    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoOut;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final DutyCycleOut dutyCycleOut;

    private final TalonFXConfiguration config;
    private final TalonFX motor;

    public ClimbIOTalonFX(RobotConfig.CANBus canBus) {
        motor = new TalonFX(ClimbConstants.MOTOR_ID, canBus.bus);
        config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.kP = ClimbConstants.kP.get();
        config.Slot0.kI = ClimbConstants.kI.get();
        config.Slot0.kD = ClimbConstants.kD.get();
        config.Slot0.kV = ClimbConstants.kV.get();
        config.Slot0.kA = ClimbConstants.kA.get();
        config.Feedback.SensorToMechanismRatio = ClimbConstants.REDUCTION;

        config.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MOTIONMAGIC_EXPO_CRUISE_VELOCITY.get();
        config.MotionMagic.MotionMagicExpo_kA = ClimbConstants.kA.get();
        config.MotionMagic.MotionMagicExpo_kV = ClimbConstants.kV.get();

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, motor::optimizeBusUtilization);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        torqueCurrent =  motor.getTorqueCurrent();
        temperature = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            position,
            velocity
        );

        PhoenixUtil.registerSignals(
            canBus.bus.isNetworkFD(),
            position,
            velocity,
            voltage,
            supplyCurrent,
            torqueCurrent,
            temperature
        );

        motionMagicExpoOut = new MotionMagicExpoTorqueCurrentFOC(0).withUpdateFreqHz(0).withSlot(0);
        voltageOut = new VoltageOut(0).withUpdateFreqHz(0);
        torqueCurrentFOC = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
        dutyCycleOut = new DutyCycleOut(0).withUpdateFreqHz(0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.connected =
            BaseStatusSignal.isAllGood(position, velocity, velocity, supplyCurrent, temperature, torqueCurrent);
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = temperature.getValueAsDouble();
    }

    @Override
    public void runPosition(Rotation2d setpoint, double feedforward) {
        motor.setControl(
            motionMagicExpoOut
                .withPosition(setpoint.getRotations())
                .withFeedForward(feedforward));
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        motor.setControl(
            isTorqueCurrent ?
                torqueCurrentFOC.withOutput(output) :
                voltageOut.withOutput(output)
        );
    }

    @Override
    public void runPercentOut(double output) {
        motor.setControl(
            dutyCycleOut.withOutput(output)
        );
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        }).start();
    }

    @Override
    public void setPIDFF(double kP, double kI, double kD, double kV, double kA, double cruiseVelocity) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.MotionMagic.MotionMagicExpo_kV = kV;
        config.MotionMagic.MotionMagicExpo_kA = kA;
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    @Override
    public void setPosition(double position) {
        new Thread(() -> {
            motor.setPosition(Units.radiansToRotations(position));
        }).start();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
