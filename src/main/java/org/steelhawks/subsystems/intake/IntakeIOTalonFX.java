package org.steelhawks.subsystems.intake;

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
import edu.wpi.first.units.measure.*;


public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX left_motor;
    private final TalonFX right_motor;

    private final TalonFX intake_motor;

    private final StatusSignal<Angle> rightExtensionPosition;
    private final StatusSignal<AngularVelocity> rightExtensionVelocityPerSec;
    private final StatusSignal<Current> rightExtensionCurrentAmps;
    private final StatusSignal<Voltage> rightExtensionAppliedVoltage;
    private final StatusSignal<Temperature> rightExtensionTemp;

    private final StatusSignal<Angle> leftExtensionPosition;
    private final StatusSignal<AngularVelocity> leftExtensionVelocityPerSec;
    private final StatusSignal<Current> leftExtensionCurrentAmps;
    private final StatusSignal<Voltage> leftExtensionAppliedVoltage;
    private final StatusSignal<Temperature> leftExtensionTemp;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocityPerSec;
    private final StatusSignal<Current> rollerCurrentAmps;
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
                .withKA(IntakeConstants.EXTENSION_VELOCITY_KA.getAsDouble())
                .withKA(IntakeConstants.EXTENSION_VELOCITY_KA.getAsDouble())
                .withKA(IntakeConstants.EXTENSION_VELOCITY_KA.getAsDouble());

        var rollerSlot0Configs = new Slot0Configs();
        rollerSlot0Configs
                .withKA(IntakeConstants.ROLLER_KA.getAsDouble())
                .withKP(IntakeConstants.ROLLER_KP.getAsDouble())
                .withKA(IntakeConstants.ROLLER_KA.getAsDouble())
                .withKA(IntakeConstants.ROLLER_KA.getAsDouble())
                .withKA(IntakeConstants.ROLLER_KA.getAsDouble());

    }


}
