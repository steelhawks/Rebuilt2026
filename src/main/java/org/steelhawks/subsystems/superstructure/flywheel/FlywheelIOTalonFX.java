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
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Current> leftTorqueCurrent;
    private final StatusSignal<Temperature> leftTemp;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;

    private final VelocityVoltage velocityVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final TalonFXConfiguration config;
    private final TalonFX leftMotor, rightMotor;

    public FlywheelIOTalonFX(CANBus bus, SubsystemConstants.FlywheelConstants constants) {
        leftMotor = new TalonFX(constants.leftMotorId(), bus);
        rightMotor = new TalonFX(constants.rightMotorId(), bus);
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = constants.reduction();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
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

        position = rightMotor.getPosition();
        velocity = rightMotor.getVelocity();
        voltage = rightMotor.getMotorVoltage();
        current = rightMotor.getSupplyCurrent();
        torqueCurrent = rightMotor.getTorqueCurrent();
        temp = rightMotor.getDeviceTemp();

        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.Feedback.SensorToMechanismRatio = constants.reduction();
        rightConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        rightConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        rightMotor.getConfigurator().apply(rightConfig);

        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withSlot(0);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);


        BaseStatusSignal.setUpdateFrequencyForAll(
            1000, position, velocity, voltage, torqueCurrent, leftPosition, leftCurrent, leftTorqueCurrent, leftVoltage);
        PhoenixUtil.registerSignals(
            bus,
            position, velocity, voltage, current, torqueCurrent, temp, leftPosition, leftCurrent, leftTorqueCurrent, leftVoltage, leftTemp);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leftConnected = BaseStatusSignal.isAllGood(leftPosition, leftCurrent, leftTorqueCurrent, leftVoltage, leftTemp);
        inputs.leftPositionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.leftAppliedVolts = voltage.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = current.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = temp.getValueAsDouble();

        inputs.rightConnected = BaseStatusSignal.isAllGood(position, velocity, voltage, current, torqueCurrent, temp);
        inputs.rightPositionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.rightAppliedVolts = voltage.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = current.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = temp.getValueAsDouble();
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
