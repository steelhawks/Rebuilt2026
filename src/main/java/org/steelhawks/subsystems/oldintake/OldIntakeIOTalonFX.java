package org.steelhawks.subsystems.oldintake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import org.steelhawks.RobotConfig.CANBusList;
import org.steelhawks.util.PhoenixUtil;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class OldIntakeIOTalonFX implements OldIntakeIO {
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

    private final PositionTorqueCurrentFOC positionTorqueCurrentFoc;
    private final TorqueCurrentFOC pivotTorqueCurrentFOC;
    private final VoltageOut pivotVoltageOut;
    private final DutyCycleOut pivotDutyCycleOut;

    private final DutyCycleOut intakeDutyCycleOut;

    private StatusSignal<Angle> encoderAngle;
    private StatusSignal<AngularVelocity> encoderVelocity;
    private StatusSignal<Voltage> encoderVolts;

    private TalonFXConfiguration leftConfig;
    private TalonFXConfiguration rightConfig;
    private TalonFXConfiguration intakeConfig;
    private CANcoderConfiguration encoderConfig;

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX intakeMotor;
    private final CANcoder encoder;

    public OldIntakeIOTalonFX(CANBus bus) {
        leftMotor = new TalonFX(OldIntakeConstants.LEFT_MOTOR_ID, bus);
        rightMotor = new TalonFX(OldIntakeConstants.RIGHT_MOTOR_ID, bus);
        intakeMotor = new TalonFX(OldIntakeConstants.INTAKE_MOTOR_ID, bus);
        encoder = new CANcoder(OldIntakeConstants.ENCODER_ID, bus);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        leftConfig = new TalonFXConfiguration();
        rightConfig = new TalonFXConfiguration();
        intakeConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.Slot0.kP = OldIntakeConstants.kP.get();
        leftConfig.Slot0.kI = OldIntakeConstants.kI.get();
        leftConfig.Slot0.kD = OldIntakeConstants.kD.get();
        leftConfig.Feedback.SensorToMechanismRatio = OldIntakeConstants.REDUCTION;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(intakeConfig));

        encoderConfig.MagnetSensor.MagnetOffset = OldIntakeConstants.MAG_OFFSET.getRotations();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig));

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightPosition = leftMotor.getPosition();
        rightVelocity = leftMotor.getVelocity();
        rightVoltage = leftMotor.getMotorVoltage();
        rightCurrent = leftMotor.getSupplyCurrent();
        rightTorqueCurrent = leftMotor.getTorqueCurrent();
        rightTemp = leftMotor.getDeviceTemp();

        intakePosition = leftMotor.getPosition();
        intakeVelocity = leftMotor.getVelocity();
        intakeVoltage = leftMotor.getMotorVoltage();
        intakeCurrent = leftMotor.getSupplyCurrent();
        intakeTorqueCurrent = leftMotor.getTorqueCurrent();
        intakeTemp = leftMotor.getDeviceTemp();

        encoderVelocity = encoder.getVelocity();
        encoderAngle = encoder.getAbsolutePosition();
        encoderVolts = encoder.getSupplyVoltage();

        tryUntilOk(5, leftMotor::optimizeBusUtilization);
        tryUntilOk(5, rightMotor::optimizeBusUtilization);
        tryUntilOk(5, intakeMotor::optimizeBusUtilization);
        tryUntilOk(5, encoder::optimizeBusUtilization);

        positionTorqueCurrentFoc = new PositionTorqueCurrentFOC(0).withSlot(0).withUpdateFreqHz(0);
        pivotTorqueCurrentFOC = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
        pivotVoltageOut = new VoltageOut(0).withUpdateFreqHz(0);
        pivotDutyCycleOut = new DutyCycleOut(0).withUpdateFreqHz(0);
        intakeDutyCycleOut = new DutyCycleOut(0).withUpdateFreqHz(0);

        // critical status signsls to 100hZ
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            leftPosition,
            leftVelocity,
            leftVoltage,
            leftCurrent,
            leftTorqueCurrent,
            leftTemp,
            rightPosition,
            rightVelocity,
            rightVoltage,
            rightCurrent,
            rightTorqueCurrent,
            rightTemp,
            intakePosition,
            intakeVelocity,
            intakeVoltage,
            intakeCurrent,
            intakeTorqueCurrent,
            intakeTemp,
            encoderAngle,
            encoderVelocity,
            encoderVolts
        );

        PhoenixUtil.registerSignals(
            bus.isNetworkFD(),
            leftPosition,
            leftVelocity,
            leftVoltage,
            leftCurrent,
            leftTorqueCurrent,
            leftTemp,
            rightPosition,
            rightVelocity,
            rightVoltage,
            rightCurrent,
            rightTorqueCurrent,
            rightTemp,
            intakePosition,
            intakeVelocity,
            intakeVoltage,
            intakeCurrent,
            intakeTorqueCurrent,
            intakeTemp,
            encoderAngle,
            encoderVelocity,
            encoderVolts
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftConnected = BaseStatusSignal.isAllGood(leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp);
        inputs.leftPositionRad = Rotation2d.fromRotations(leftPosition.getValueAsDouble());
        inputs.leftVelocityRadPerSec = leftVelocity.getValueAsDouble();
        inputs.leftAppliedVolts = leftVelocity.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected = BaseStatusSignal.isAllGood(rightPosition, rightVelocity, rightPosition, rightCurrent, rightTorqueCurrent, rightCurrent);
        inputs.rightPositionRad = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
        inputs.rightVelocityRadPerSec = rightVelocity.getValueAsDouble();
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();

        inputs.intakeConnected = BaseStatusSignal.isAllGood(intakePosition, intakeVelocity, intakeVoltage, intakeCurrent, intakeTorqueCurrent, intakeTemp);
        inputs.intakePositionRad = Rotation2d.fromRotations(intakePosition.getValueAsDouble());
        inputs.intakeVelocityRadPerSec = intakeVelocity.getValueAsDouble();
        inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
        inputs.intakeSupplyCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.intakeTorqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
        inputs.intakeTempCelsius = intakeTemp.getValueAsDouble();

        inputs.encConnected = BaseStatusSignal.isAllGood(encoderVolts, encoderVelocity, encoderAngle);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            leftConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));
        }).start();
    }

    @Override
    public void runPivotPosition(double position, double feedforward) {
        leftMotor.setControl(
            positionTorqueCurrentFoc
                .withPosition(Units.radiansToRotations(position))
                .withFeedForward(feedforward)
        );
    }

    @Override
    public void runPivotOpenLoop(double output, boolean isTorqueCurrent) {
        leftMotor.setControl(
            isTorqueCurrent ?
                pivotTorqueCurrentFOC.withOutput(output)
                : pivotVoltageOut.withOutput(output)
        );
    }

    @Override
    public void runPivotPercentOut(double output) {
        leftMotor.setControl(pivotDutyCycleOut.withOutput(output));
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
        intakeMotor.setControl(intakeDutyCycleOut.withOutput(output));
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    @Override
    public void stopPivot() {
        leftMotor.stopMotor();
    }
}
