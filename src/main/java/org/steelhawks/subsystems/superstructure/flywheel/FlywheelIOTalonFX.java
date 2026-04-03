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
import org.steelhawks.RobotConfig;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

    private final StatusSignal<Angle> leftposition;
    private final StatusSignal<AngularVelocity> leftvelocity;
    private final StatusSignal<Voltage> leftvoltage;
    private final StatusSignal<Current> leftcurrent;
    private final StatusSignal<Current> lefttorqueCurrent;
    private final StatusSignal<Temperature> lefttemp;

    private final StatusSignal<Angle> rightposition;
    private final StatusSignal<AngularVelocity> rightvelocity;
    private final StatusSignal<Voltage> rightvoltage;
    private final StatusSignal<Current> rightcurrent;
    private final StatusSignal<Current> righttorqueCurrent;
    private final StatusSignal<Temperature> righttemp;

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

        leftposition = leftMotor.getPosition();
        leftvelocity = leftMotor.getVelocity();
        leftvoltage = leftMotor.getMotorVoltage();
        leftcurrent = leftMotor.getSupplyCurrent();
        lefttorqueCurrent = leftMotor.getTorqueCurrent();
        lefttemp = leftMotor.getDeviceTemp();

        rightMotor = new TalonFX(constants.rightMotorId(), bus);
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

        rightposition = rightMotor.getPosition();
        rightvelocity = rightMotor.getVelocity();
        rightvoltage = rightMotor.getMotorVoltage();
        rightcurrent = rightMotor.getSupplyCurrent();
        righttorqueCurrent = rightMotor.getTorqueCurrent();
        righttemp = rightMotor.getDeviceTemp();

        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withSlot(0);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);


        BaseStatusSignal.setUpdateFrequencyForAll(
            1000, leftposition, leftvelocity, leftvoltage, lefttorqueCurrent, rightposition, rightvelocity, rightvoltage, righttorqueCurrent);
        PhoenixUtil.registerSignals(
            bus,
            leftposition, leftvelocity, leftvoltage, leftcurrent, lefttorqueCurrent, lefttemp, rightposition, rightvelocity, rightvoltage, rightcurrent, righttorqueCurrent, righttemp);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leftConnected = BaseStatusSignal.isAllGood(leftposition, leftvelocity, leftvoltage, leftcurrent, lefttorqueCurrent, lefttemp);
        inputs.leftPositionRad = Units.rotationsToRadians(leftposition.getValueAsDouble());
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftvelocity.getValueAsDouble());
        inputs.leftAppliedVolts = leftvoltage.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftcurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = lefttorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = lefttemp.getValueAsDouble();

        inputs.rightConnected = BaseStatusSignal.isAllGood(rightposition, rightvelocity, rightvoltage, rightcurrent, righttorqueCurrent, righttemp);
        inputs.rightPositionRad = Units.rotationsToRadians(rightposition.getValueAsDouble());
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightvelocity.getValueAsDouble());
        inputs.rightAppliedVolts = rightvoltage.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightcurrent.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = righttorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = righttemp.getValueAsDouble();
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
