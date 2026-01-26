package org.steelhawks.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig.CANBus;
import org.steelhawks.util.PhoenixUtil;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class IntakeIOTalonFX implements IntakeIO {

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

    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<AngularVelocity> intakeVelocity;
    private final StatusSignal<Voltage> intakeVoltage;
    private final StatusSignal<Current> intakeCurrent;
    private final StatusSignal<Current> intakeTorqueCurrent;
    private final StatusSignal<Temperature> intakeTemp;

    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;
    private final StatusSignal<Voltage> encoderVoltage;

    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final TorqueCurrentFOC pivotTorqueCurrentFOC;
    private final DutyCycleOut pivotDutyCycleOut;
    private final VoltageOut pivotVoltageOut;

    private final DutyCycleOut intakeDutyCycleOut;

    private final TalonFXConfiguration leftConfig;
    private final TalonFXConfiguration rightConfig;
    private final TalonFXConfiguration intakeConfig;
    private final CANcoderConfiguration cancoderConfig;

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX intakeMotor;
    private final CANcoder pivotEncoder;

    public IntakeIOTalonFX(CANBus canBus) {
        leftMotor = new TalonFX(IntakeConstants.LEFT_MOTOR_ID, canBus.bus);
        rightMotor = new TalonFX(IntakeConstants.RIGHT_MOTOR_ID, canBus.bus);
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, canBus.bus);
        pivotEncoder = new CANcoder(IntakeConstants.ENCODER_ID, canBus.bus);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        leftConfig = new TalonFXConfiguration();
        rightConfig = new TalonFXConfiguration();
        intakeConfig = new TalonFXConfiguration();
        cancoderConfig = new CANcoderConfiguration();

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.Slot0.kP = IntakeConstants.kP.getAsDouble();
        leftConfig.Slot0.kI = IntakeConstants.kI.getAsDouble();
        leftConfig.Slot0.kD = IntakeConstants.kD.getAsDouble();
        leftConfig.Feedback.SensorToMechanismRatio = IntakeConstants.REDUCTION;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));

        cancoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.MAG_OFFSET.getRotations();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // TODO FIND
        tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(cancoderConfig));

        tryUntilOk(5, leftMotor::optimizeBusUtilization);
        tryUntilOk(5, rightMotor::optimizeBusUtilization);
        tryUntilOk(5, intakeMotor::optimizeBusUtilization);
        tryUntilOk(5, pivotEncoder::optimizeBusUtilization);

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getMotorVoltage();
        leftCurrent = leftMotor.getStatorCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightVoltage = rightMotor.getMotorVoltage();
        rightCurrent = rightMotor.getStatorCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        intakePosition = intakeMotor.getPosition();
        intakeVelocity = intakeMotor.getVelocity();
        intakeVoltage = intakeMotor.getMotorVoltage();
        intakeCurrent = intakeMotor.getStatorCurrent();
        intakeTorqueCurrent = intakeMotor.getTorqueCurrent();
        intakeTemp = intakeMotor.getDeviceTemp();

        encoderPosition = pivotEncoder.getAbsolutePosition();
        encoderVelocity = pivotEncoder.getVelocity();
        encoderVoltage = pivotEncoder.getSupplyVoltage();

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        pivotTorqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        pivotDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
        pivotVoltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        intakeDutyCycleOut = new DutyCycleOut(0.0);

        PhoenixUtil.registerSignals(
            canBus.bus.isNetworkFD(),
            leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp,
            rightPosition, rightVelocity, rightVelocity, rightCurrent, rightTorqueCurrent, rightTemp,
            intakePosition, intakeVelocity, intakeCurrent, intakeTorqueCurrent, intakeTemp,
            encoderPosition, encoderVelocity, encoderVoltage
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftConnected =
            BaseStatusSignal.isAllGood(leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp);
        inputs.leftPositionRad = Rotation2d.fromRotations(leftPosition.getValueAsDouble());
        inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected =
            BaseStatusSignal.isAllGood(rightPosition, rightVelocity, rightVelocity, rightCurrent, rightTorqueCurrent, rightTemp);
        inputs.rightPositionRad = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
        inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();

        inputs.intakeConnected =
            BaseStatusSignal.isAllGood(intakePosition, intakeVelocity, intakeCurrent, intakeTorqueCurrent, intakeTemp);
        inputs.intakePositionRad = Rotation2d.fromRotations(intakePosition.getValueAsDouble());
        inputs.intakeVelocityRadPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
        inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.intakeTorqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
        inputs.intakeTempCelsius = intakeTemp.getValueAsDouble();

        inputs.encConnected = BaseStatusSignal.isAllGood(encoderPosition, encoderVelocity, encoderVoltage);
        inputs.encAbsPositionRad = Rotation2d.fromRotations(encoderPosition.getValueAsDouble());
        inputs.encVelocityRadPerSec = Units.rotationsToRadians(encoderVelocity.getValueAsDouble());
        inputs.encAppliedVolts = encoderVoltage.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            leftConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));
        }).start();
    }

    @Override
    public void runPivotPosition(double position, double feedforward) {
        leftMotor.setControl(
            positionTorqueCurrentFOC
                .withPosition(position)
                .withFeedForward(feedforward));
    }

    @Override
    public void runPivotOpenLoop(double output, boolean isTorqueCurrent) {
        leftMotor.setControl(
            isTorqueCurrent
                ? pivotTorqueCurrentFOC.withOutput(output)
                : pivotVoltageOut.withOutput(output));
    }

    @Override
    public void runPivotPercentOut(double output) {
        leftMotor.setControl(
            pivotDutyCycleOut.withOutput(output));
    }

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        leftConfig.Slot0.kP = kP;
        leftConfig.Slot0.kI = kI;
        leftConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));
    }

    @Override
    public void runIntake(double output) {
        intakeMotor.setControl(
            intakeDutyCycleOut.withOutput(output));
    }

    @Override
    public void stopPivot() {
        leftMotor.stopMotor();
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }
}
