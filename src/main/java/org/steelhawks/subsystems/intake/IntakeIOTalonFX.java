package org.steelhawks.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;


public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX left_motor;
    private final TalonFX right_motor;

    private final TalonFX intake_motor;

    private final StatusSignal<Angle> rightExtensionPosition;
    private final StatusSignal<AngularVelocity> rightExtensionVelocityPerSec;
    private final StatusSignal<Current> rightExtensionCurrentAmps;
    private final StatusSignal<Current> rightExtensionTorqueCurrent;
    private final StatusSignal<Current> rightExtensionStatorCurrent;
    private final StatusSignal<Voltage> rightExtensionAppliedVoltage;
    private final StatusSignal<Temperature> rightExtensionTemp;

    private final StatusSignal<Angle> leftExtensionPosition;
    private final StatusSignal<AngularVelocity> leftExtensionVelocityPerSec;
    private final StatusSignal<Current> leftExtensionCurrentAmps;
    private final StatusSignal<Current> leftExtensionTorqueCurrent;
    private final StatusSignal<Current> leftExtensionStatorCurrent;
    private final StatusSignal<Voltage> leftExtensionAppliedVoltage;
    private final StatusSignal<Temperature> leftExtensionTemp;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocityPerSec;
    private final StatusSignal<Current> rollerCurrentAmps;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Current> rollerStatorCurrent;
    private final StatusSignal<Voltage> rollerAppliedVoltage;
    private final StatusSignal<Temperature> rollerTemp;

    private final VoltageOut extensionVoltage;
    private final VelocityVoltage extensionVelocityVoltage;
    private final PositionVoltage extensionPositionVoltage;

    private final VoltageOut rollerVoltage;
    private final VelocityVoltage rollerVelocityVoltage;

    private boolean isHomed = false;

    public IntakeIOTalonFX() {

        // for both of these we need canbus id
        left_motor = new TalonFX(IntakeConstants.EXTENSION_LEFT_MOTOR_ID);
        right_motor = new TalonFX(IntakeConstants.EXTENSION_RIGHT_MOTOR_ID);
        intake_motor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

        extensionVoltage = new VoltageOut(0).withEnableFOC(true);
        extensionVelocityVoltage = new VelocityVoltage(0).withEnableFOC(true);
        extensionPositionVoltage = new PositionVoltage(0).withEnableFOC(true);

        rollerVoltage = new VoltageOut(0).withEnableFOC(true);
        rollerVelocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

        left_motor.setControl(new Follower(IntakeConstants.EXTENSION_RIGHT_MOTOR_ID, MotorAlignmentValue.Opposed));

        var extensionConfig = new TalonFXConfiguration();
        extensionConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.EXTENSION_CURRENT_LIMIT.getAsDouble();
        extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        extensionConfig.MotorOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        extensionConfig.Feedback.SensorToMechanismRatio = IntakeConstants.EXTENSION_GEAR_RATIO.getAsDouble();

        left_motor.getConfigurator().apply(extensionConfig);

        var extensionSlot0Configs = new Slot0Configs();
        extensionSlot0Configs
                .withKA(IntakeConstants.EXTENSION_POSITION_KA.getAsDouble())
                .withKP(IntakeConstants.EXTENSION_POSITION_KP.getAsDouble())
                .withKI(IntakeConstants.EXTENSION_POSITION_KI.getAsDouble())
                .withKD(IntakeConstants.EXTENSION_POSITION_KD.getAsDouble())
                .withKS(IntakeConstants.EXTENSION_POSITION_KS.getAsDouble())
                .withKV(IntakeConstants.EXTENSION_POSITION_KV.getAsDouble())
                .withKG(IntakeConstants.EXTENSION_POSITION_KG.getAsDouble());
        var extensionSlot1Configs = new Slot1Configs();
        extensionSlot1Configs
                .withKA(IntakeConstants.EXTENSION_VELOCITY_KA.getAsDouble())
                .withKP(IntakeConstants.EXTENSION_VELOCITY_KP.getAsDouble())
                .withKI(IntakeConstants.EXTENSION_VELOCITY_KI.getAsDouble())
                .withKD(IntakeConstants.EXTENSION_VELOCITY_KD.getAsDouble())
                .withKV(IntakeConstants.EXTENSION_VELOCITY_KV.getAsDouble())
                .withKS(IntakeConstants.EXTENSION_VELOCITY_KS.getAsDouble())
                .withKG(IntakeConstants.EXTENSION_VELOCITY_KG.getAsDouble());

        left_motor.getConfigurator().apply(extensionSlot0Configs);
        left_motor.getConfigurator().apply(extensionSlot1Configs);

        var rollerConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive));

        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_CURRENT_LIMIT.getAsDouble();
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intake_motor.getConfigurator().apply(rollerConfig);

        var rollerSlot0Configs = new Slot0Configs();
        rollerSlot0Configs
                .withKA(IntakeConstants.ROLLER_KA.getAsDouble())
                .withKP(IntakeConstants.ROLLER_KP.getAsDouble())
                .withKI(IntakeConstants.ROLLER_KI.getAsDouble())
                .withKD(IntakeConstants.ROLLER_KD.getAsDouble())
                .withKS(IntakeConstants.ROLLER_KS.getAsDouble())
                .withKV(IntakeConstants.ROLLER_KV.getAsDouble());

        intake_motor.getConfigurator().apply(rollerSlot0Configs);

        rightExtensionPosition = right_motor.getPosition();
        rightExtensionAppliedVoltage = right_motor.getMotorVoltage();
        rightExtensionVelocityPerSec = right_motor.getVelocity();
        rightExtensionCurrentAmps = right_motor.getSupplyCurrent();
        rightExtensionStatorCurrent = right_motor.getStatorCurrent();
        rightExtensionTorqueCurrent = right_motor.getTorqueCurrent();
        rightExtensionTemp = right_motor.getDeviceTemp();


        leftExtensionPosition = left_motor.getPosition();
        leftExtensionAppliedVoltage = left_motor.getMotorVoltage();
        leftExtensionVelocityPerSec = left_motor.getVelocity();
        leftExtensionTorqueCurrent = left_motor.getTorqueCurrent();
        leftExtensionStatorCurrent = left_motor.getStatorCurrent();
        leftExtensionCurrentAmps = left_motor.getSupplyCurrent();
        leftExtensionTemp = left_motor.getDeviceTemp();

        rollerPosition = intake_motor.getPosition();
        rollerAppliedVoltage = intake_motor.getMotorVoltage();
        rollerVelocityPerSec = intake_motor.getVelocity();
        rollerStatorCurrent = intake_motor.getStatorCurrent();
        rollerTorqueCurrent = intake_motor.getTorqueCurrent();
        rollerCurrentAmps = intake_motor.getSupplyCurrent();
        rollerTemp = intake_motor.getDeviceTemp();

        PhoenixUtil.registerSignals(RobotConfig.CANBusList.kIntakeBus,
                rightExtensionPosition,
                rightExtensionAppliedVoltage,
                rightExtensionTemp,
                rightExtensionCurrentAmps,
                rightExtensionTorqueCurrent,
                rightExtensionStatorCurrent,
                rightExtensionVelocityPerSec,

                leftExtensionAppliedVoltage,
                leftExtensionPosition,
                leftExtensionCurrentAmps,
                leftExtensionTorqueCurrent,
                leftExtensionStatorCurrent,
                leftExtensionTemp,
                leftExtensionVelocityPerSec,

                rollerTemp,
                rollerAppliedVoltage,
                rollerCurrentAmps,
                rollerTorqueCurrent,
                rollerStatorCurrent,
                rollerPosition,
                rollerVelocityPerSec
                );

        right_motor.optimizeBusUtilization();
        left_motor.optimizeBusUtilization();
        intake_motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftConnected = PhoenixUtil.refreshAll();
        inputs.leftExtensionPosition = leftExtensionPosition.getValueAsDouble() * IntakeConstants.PINION_ROTATION;
        inputs.leftExtensionVelocity = leftExtensionVelocityPerSec.getValueAsDouble() * IntakeConstants.PINION_ROTATION;
        inputs.leftExtensionAppliedVolts = leftExtensionAppliedVoltage.getValueAsDouble();
        inputs.leftExtensionCurrentAmps = leftExtensionCurrentAmps.getValueAsDouble();
        inputs.leftExtensionTorqueCurrent = leftExtensionTorqueCurrent.getValueAsDouble();
        inputs.leftExtensionStatorCurrent = leftExtensionStatorCurrent.getValueAsDouble();
        inputs.leftExtensionTempCelsius = leftExtensionTemp.getValueAsDouble();

        inputs.rightConnected = BaseStatusSignal.isAllGood(rightExtensionTemp, rightExtensionCurrentAmps, rightExtensionVelocityPerSec, rightExtensionPosition, rightExtensionAppliedVoltage);
        inputs.rightExtensionPosition = rightExtensionPosition.getValueAsDouble() * IntakeConstants.PINION_ROTATION;
        inputs.rightExtensionVelocity = rightExtensionVelocityPerSec.getValueAsDouble() * IntakeConstants.PINION_ROTATION;
        inputs.rightExtensionAppliedVolts = rightExtensionAppliedVoltage.getValueAsDouble();
        inputs.rightExtensionCurrentAmps = rightExtensionCurrentAmps.getValueAsDouble();
        inputs.rightExtensionTorqueCurrent = rightExtensionTorqueCurrent.getValueAsDouble();
        inputs.rightExtensionStatorCurrent = rightExtensionStatorCurrent.getValueAsDouble();
        inputs.rightExtensionTempCelsius = leftExtensionTemp.getValueAsDouble();

        inputs.rollerConnected = BaseStatusSignal.isAllGood(rollerPosition, rollerVelocityPerSec, rollerTemp, rollerCurrentAmps, rollerAppliedVoltage);
        inputs.rollerPosition = new Rotation2d(rollerPosition.getValueAsDouble());
        inputs.rollerVelocity = rollerVelocityPerSec.getValueAsDouble();
        inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrentAmps.getValueAsDouble();
        inputs.rollerTorqueCurrent = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerStatorCurrent = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerTorqueCurrent = rollerStatorCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();

        inputs.isExtended = inputs.rightExtensionPosition >= IntakeConstants.MAX_EXTENSION - IntakeConstants.POSITION_TOLERANCE;
        inputs.isRetracted = inputs.rightExtensionPosition <= IntakeConstants.MIN_EXTENSION + IntakeConstants.POSITION_TOLERANCE;
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        left_motor.setControl(extensionVoltage.withOutput(voltage));
    }

    @Override
    public void setExtensionVelocity(double velocityPerSec, double ffOutput) {
        left_motor.setControl(extensionVelocityVoltage
                .withVelocity(velocityPerSec / IntakeConstants.PINION_ROTATION)
                .withFeedForward(ffOutput)
                .withSlot(1));
    }

    @Override
    public void stopExtension() { left_motor.stopMotor(); }

    @Override
    public void setExtensionBrakeMode(boolean enable) {
        left_motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        right_motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setExtensionPosition(double positionMeters, double ffVolts) {
        left_motor.setControl(
                extensionPositionVoltage
                        .withPosition(positionMeters / IntakeConstants.PINION_ROTATION)
                        .withFeedForward(ffVolts)
                        .withSlot(0)
        );
    }

    @Override
    public void resetExtension( double positionMeters ) {
        left_motor.setPosition(positionMeters / IntakeConstants.PINION_ROTATION);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        intake_motor.setControl(rollerVoltage.withOutput(voltage));
    }

    @Override
    public void setRollerVelocity(double velocityPerSec, double ffVolts) {
        intake_motor.setControl(rollerVelocityVoltage.withVelocity(velocityPerSec / (2.0 * Math.PI)).withFeedForward(ffVolts));
    }

    @Override
    public void stopRoller() { intake_motor.stopMotor(); }

    @Override
    public void setRollerBrakeMode(boolean enable) {
        left_motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        right_motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public double getExtensionSetpoint() {
        return extensionPositionVoltage.Position * IntakeConstants.PINION_ROTATION;
    }

    @Override
    public double getExtensionVelocitySetpoint() {
        return extensionVelocityVoltage.Velocity * IntakeConstants.PINION_ROTATION;
    }

    @Override
    public double getRollerVelocitySetpoint() {
        return rollerVelocityVoltage.Velocity * (2.0 * Math.PI);
    }


}
