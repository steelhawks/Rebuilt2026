package org.steelhawks.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.steelhawks.BuilderConstants;
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

    private final TalonFXConfiguration left_config;
    private final TalonFXConfiguration right_config;
    private final TalonFXConfiguration roller_config;


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

    private final PositionTorqueCurrentFOC left_motor_torque_current;
    private final PositionTorqueCurrentFOC right_motor_torque_current;
    private final TorqueCurrentFOC left_extension_torque_current;
    private final TorqueCurrentFOC right_extension_torque_current;

    private final DutyCycleOut intakePercentOut;
    private final VoltageOut extensionVoltage;

    private final DutyCycleOut extensionPercentOut;

    private boolean isHomed = false;

    public IntakeIOTalonFX() {

        // for both of these we need canbus id
        left_motor = new TalonFX(IntakeConstants.EXTENSION_LEFT_MOTOR_ID);
        right_motor = new TalonFX(IntakeConstants.EXTENSION_RIGHT_MOTOR_ID);
        intake_motor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

        right_config = new TalonFXConfiguration();
        left_config = new TalonFXConfiguration();
        roller_config = new TalonFXConfiguration();

       left_motor_torque_current = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
       right_motor_torque_current = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);


        left_extension_torque_current = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        right_extension_torque_current = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        extensionVoltage = new VoltageOut(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);

        extensionPercentOut = new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

        intakePercentOut = new DutyCycleOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

        left_motor.setControl(new Follower(IntakeConstants.EXTENSION_RIGHT_MOTOR_ID, MotorAlignmentValue.Opposed));

        right_config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.EXTENSION_CURRENT_LIMIT.getAsDouble();
        right_config.CurrentLimits.SupplyCurrentLimitEnable = true;

        left_config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.EXTENSION_CURRENT_LIMIT.getAsDouble();
        left_config.CurrentLimits.SupplyCurrentLimitEnable = true;

        right_config.MotorOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        right_config.Feedback.SensorToMechanismRatio = IntakeConstants.EXTENSION_GEAR_RATIO.getAsDouble();

        left_config.MotorOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        left_config.Feedback.SensorToMechanismRatio = IntakeConstants.EXTENSION_GEAR_RATIO.getAsDouble();

        left_motor.getConfigurator().apply(left_config);
        right_motor.getConfigurator().apply(right_config);


        BuilderConstants.ExtensionPIDConstants.setPID(left_motor);
        BuilderConstants.ExtensionPIDConstants.setPID(right_motor);


                roller_config.withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive));

        roller_config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_CURRENT_LIMIT.getAsDouble();
        roller_config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intake_motor.getConfigurator().apply(roller_config);

        BuilderConstants.RollerPIDConstants.setPID(intake_motor);

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
    public void setExtensionPosition(double position, double Lfeedforward, double Rfeedforward) {
        double rotations = position / IntakeConstants.PINION_METERS_TO_RADIANS;
        left_motor.setControl(
                left_motor_torque_current.withPosition(rotations).withFeedForward(Lfeedforward)
        );

        right_motor.setControl(
                right_motor_torque_current.withPosition(rotations).withFeedForward(Rfeedforward)
        );
    }


    @Override
    public void stopExtension() { left_motor.stopMotor(); }


    @Override
    public void stopIntake() { intake_motor.stopMotor(); }

    @Override
    public void runExtensionOpenLoop(double output, boolean isTorqueCurrent) {
        left_motor.setControl(
                isTorqueCurrent ? left_extension_torque_current.withOutput(output) : extensionVoltage.withOutput(output)
        );

        right_motor.setControl(
                isTorqueCurrent ? right_extension_torque_current.withOutput(output) : extensionVoltage.withOutput(output)
        );
    }

    @Override
    public void runExtensionPercentOut(double output) {
        left_motor.setControl(
                extensionPercentOut.withOutput(output)
        );
    }

    @Override
    public void runIntake(double output) {
        intake_motor.setControl(
                intakePercentOut.withOutput(output)
        );
    }

    @Override
    public void setExtensionPID(double kP, double kI, double kD) {
        right_config.Slot0.kP = kP;
        right_config.Slot0.kI = kI;
        right_config.Slot0.kD = kD;

        left_config.Slot0.kP = kP;
        left_config.Slot0.kI = kI;
        left_config.Slot0.kD = kD;

        right_motor.getConfigurator().apply(right_config);
        left_motor.getConfigurator().apply(left_config);
    }

    @Override
    public void setRollerPID(double kP, double kI, double kD) {
        roller_config.Slot0.kP = kP;
        roller_config.Slot0.kI = kI;
        roller_config.Slot0.kP = kP;
    }
}
