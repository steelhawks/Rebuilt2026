package org.steelhawks.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.CurrentLimits;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;

    private final StatusSignal<Angle> cancoderPosition;
    private final StatusSignal<AngularVelocity> cancoderVelocity;
    private final StatusSignal<Voltage> cancoderVoltage;

    private final PositionVoltage positionVoltage;
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final DutyCycleOut dutyCycleOut;

    private final TalonFXConfiguration motorConfig;
    private final CANcoderConfiguration encoderConfig;
    private final TalonFX motor;
    private final CANcoder encoder;

    public TurretIOTalonFX(CANBus bus, SubsystemConstants.TurretConstants constants) {
        encoder = new CANcoder(constants.encoderId(), bus);
        encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset().getRotations();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig));

        motor = new TalonFX(constants.turretId(), bus);
        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.Slot0.kP = constants.kP();
        motorConfig.Slot0.kI = constants.kI();
        motorConfig.Slot0.kD = constants.kD();
        motorConfig.Slot0.kS = constants.kS();
        motorConfig.ClosedLoopGeneral.ContinuousWrap = false;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.RotorToSensorRatio = constants.motorReduction();
        motorConfig.Feedback.SensorToMechanismRatio = 6.0 / 7.0;
        motorConfig.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.turretCurrent;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.SupplyLimit.turretEnabled;
        motorConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.turretCurrent;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = CurrentLimits.StatorLimit.turretEnabled;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(constants.maxVelocityRadPerSec());
        motorConfig.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(constants.maxAccelerationRadPerSecSq());
        motorConfig.MotionMagic.MotionMagicJerk = 0.0;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
        PhoenixUtil.tryUntilOk(5, motor::optimizeBusUtilization);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        current = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temp = motor.getDeviceTemp();

        cancoderPosition = encoder.getAbsolutePosition();
        cancoderVelocity = encoder.getVelocity();
        cancoderVoltage = encoder.getSupplyVoltage();

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withSlot(0);
        positionVoltage = new PositionVoltage(0.0).withSlot(1);
        motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0.0);
        voltageOut = new VoltageOut(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0);
        dutyCycleOut = new DutyCycleOut(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            100, voltage, current, torqueCurrent, temp);
        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            position, velocity, cancoderPosition, cancoderVelocity, cancoderVoltage);
        PhoenixUtil.registerSignals(
            bus,
            position, velocity, voltage, current, torqueCurrent, temp,
            cancoderPosition, cancoderVelocity, cancoderVoltage);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, voltage, current, torqueCurrent, temp);
        inputs.positionRad = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.velocityRadPerSec = Rotation2d.fromRotations(velocity.getValueAsDouble());
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrentAmps = current.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();

        inputs.encoderConnected = BaseStatusSignal.isAllGood(cancoderPosition, cancoderVelocity, cancoderVoltage);
        inputs.encoderPositionRad = Rotation2d.fromRotations(cancoderPosition.getValueAsDouble());
        inputs.encoderVelocityRadPerSec = Units.rotationsToRadians(cancoderVelocity.getValueAsDouble());
        inputs.encoderAppliedVolts = cancoderVoltage.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
        }).start();
    }

    @Override
    public void runPivot(double setpoint, double feedforward) {
        motor.setControl(
            positionTorqueCurrentFOC.withPosition(Units.radiansToRotations(setpoint))
                .withFeedForward(feedforward));
    }

    @Override
    public void runPivotMM(double setpoint, double feedforward) {
        motor.setControl(
            motionMagicTorqueCurrentFOC.withPosition(Units.radiansToRotations(setpoint))
                .withFeedForward(feedforward));
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        motor.setControl(
            isTorqueCurrent
                ? torqueCurrentFOC.withOutput(output)
                : voltageOut.withOutput(output));
    }

    @Override
    public void runPercentOutput(double output) {
        motor.setControl(
            dutyCycleOut.withOutput(output));
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        motorConfig.Slot0.kP = kp;
        motorConfig.Slot0.kI = ki;
        motorConfig.Slot0.kD = kd;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
    }

    @Override
    public void setMotionMagic(double cruiseVelocity, double accel, double jerk) {
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Units.radiansToRotations(cruiseVelocity);
        motorConfig.MotionMagic.MotionMagicAcceleration = Units.radiansToRotations(accel);
        motorConfig.MotionMagic.MotionMagicJerk = Units.radiansToRotations(jerk);
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
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
