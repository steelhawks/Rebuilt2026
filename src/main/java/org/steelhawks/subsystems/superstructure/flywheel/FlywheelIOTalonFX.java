package org.steelhawks.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.CurrentLimits;
import org.steelhawks.RobotConfig;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Current> leftTorqueCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Voltage> rightVoltage;
    private final StatusSignal<Current> rightCurrent;
    private final StatusSignal<Current> rightTorqueCurrent;
    private final StatusSignal<Temperature> rightTemp;

    private final VelocityVoltage velocityVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final TalonFXConfiguration config;
    private final TalonFX leftMotor, rightMotor;

    public FlywheelIOTalonFX(CANBus bus, SubsystemConstants.FlywheelConstants constants) {
        leftMotor = new TalonFX(constants.leftMotorId(), bus);
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = constants.reduction();
        config.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.flywheelCurrent;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.flywheelCurrent;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.Slot0.kP = constants.kP();
        config.Slot0.kI = constants.kI();
        config.Slot0.kD = constants.kD();
        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightMotor = new TalonFX(constants.rightMotorId(), bus);
        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.Feedback.SensorToMechanismRatio = constants.reduction();
        rightConfig.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.flywheelCurrent;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.flywheelCurrent;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        rightMotor.getConfigurator().apply(rightConfig);

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightVoltage = rightMotor.getMotorVoltage();
        rightCurrent = rightMotor.getSupplyCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withSlot(0);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);


        BaseStatusSignal.setUpdateFrequencyForAll(
            100, leftPosition, leftVelocity, leftVoltage, leftTorqueCurrent, rightPosition, rightVelocity, rightVoltage, rightTorqueCurrent);
        PhoenixUtil.registerSignals(
            bus,
            leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp, rightPosition, rightVelocity, rightVoltage, rightCurrent, rightTorqueCurrent, rightTemp);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leftConnected = BaseStatusSignal.isAllGood(leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp);
        inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected = BaseStatusSignal.isAllGood(rightPosition, rightVelocity, rightVoltage, rightCurrent, rightTorqueCurrent, rightTemp);
        inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();
    }

    @Override
    public void runFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {
        setpoint = Units.radiansToRotations(setpoint);
        leftMotor.setControl(
            isTorqueCurrent
                ? velocityTorqueCurrentFOC.withVelocity(setpoint)
                    .withFeedForward(feedforward)
                : velocityVoltage.withVelocity(setpoint)
                    .withFeedForward(feedforward)
        );
    }

    @Override
    public void runFlywheelOpenLoop(double output, boolean isTorqueCurrent) {
        leftMotor.setControl(
            isTorqueCurrent
                ? torqueCurrentFOC.withOutput(output)
                : voltageOut.withOutput(output)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
    }
}
