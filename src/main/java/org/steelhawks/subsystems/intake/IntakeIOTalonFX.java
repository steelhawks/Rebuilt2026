package org.steelhawks.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.CurrentLimits;
import org.steelhawks.SubsystemConstants;
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

    private final PositionTorqueCurrentFOC leftPositionTorqueCurrentFOC;
    private final PositionTorqueCurrentFOC rightPositionTorqueCurrentFOC;
    private final TorqueCurrentFOC leftRackTorqueCurrentFOC;
    private final TorqueCurrentFOC rightRackTorqueCurrentFOC;
    private final DutyCycleOut leftRackDutyCycleOut;
    private final DutyCycleOut rightRackDutyCycleOut;
    private final VoltageOut leftRackVoltageOut;
    private final VoltageOut rightRackVoltageOut;

    private final MotionMagicVoltage m_request;
    private final VoltageOut m_voltageOut;

    private final DutyCycleOut intakeDutyCycleOut;

    private final TalonFXConfiguration leftConfig;
    private final TalonFXConfiguration rightConfig;
    private final TalonFXConfiguration intakeConfig;

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX intakeMotor;

    public IntakeIOTalonFX(CANBus bus, SubsystemConstants.IntakeConstants constants) {
        leftMotor = new TalonFX(constants.leftId(), bus);
        rightMotor = new TalonFX(constants.rightId(), bus);
        intakeMotor = new TalonFX(constants.intakeId(), bus);

        leftConfig = new TalonFXConfiguration();
        rightConfig = new TalonFXConfiguration();
        intakeConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.Slot0.kP = constants.kP();
        leftConfig.Slot0.kI = constants.kI();
        leftConfig.Slot0.kD = constants.kD();
        leftConfig.Slot0.kS = constants.kS();
        leftConfig.Slot0.kA = constants.kA();
        leftConfig.Slot0.kG = constants.kG();
        leftConfig.Slot0.GravityType = constants.gravityType();
        leftConfig.Feedback.SensorToMechanismRatio = IntakeConstants.REDUCTION;

        leftConfig.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.intakePositionCurrent;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.intakePositionCurrent;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftConfig.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity();
        leftConfig.MotionMagic.MotionMagicAcceleration = constants.maxAccelMetersPerSecSq();
        leftConfig.MotionMagic.MotionMagicJerk = constants.jerk();

        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardSoftLimit();
        leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseSoftLimit();

        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));

        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.Slot0.kP = constants.kP();
        rightConfig.Slot0.kI = constants.kI();
        rightConfig.Slot0.kD = constants.kD();
        rightConfig.Slot0.kS = constants.kS();
        rightConfig.Slot0.kA = constants.kA();
        rightConfig.Slot0.kG = constants.kG();
        rightConfig.Slot0.GravityType = constants.gravityType();
        rightConfig.Feedback.SensorToMechanismRatio = IntakeConstants.REDUCTION;

        rightConfig.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.intakePositionCurrent;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.intakePositionCurrent;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;


        rightConfig.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity();
        rightConfig.MotionMagic.MotionMagicAcceleration = constants.maxAccelMetersPerSecSq();
        rightConfig.MotionMagic.MotionMagicJerk = constants.jerk();

        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardSoftLimit();
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseSoftLimit();

        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig));

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeConfig.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.intakeRollersCurrent;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.intakeRollersCurrent;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig));
        tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor, intakeMotor));

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftVoltage = leftMotor.getMotorVoltage();
        leftCurrent = leftMotor.getSupplyCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTemp = leftMotor.getDeviceTemp();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightVoltage = rightMotor.getMotorVoltage();
        rightCurrent = rightMotor.getSupplyCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightTemp = rightMotor.getDeviceTemp();

        intakePosition = intakeMotor.getPosition();
        intakeVelocity = intakeMotor.getVelocity();
        intakeVoltage = intakeMotor.getMotorVoltage();
        intakeCurrent = intakeMotor.getSupplyCurrent();
        intakeTorqueCurrent = intakeMotor.getTorqueCurrent();
        intakeTemp = intakeMotor.getDeviceTemp();

        leftPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        rightPositionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        leftRackTorqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        rightRackTorqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        leftRackDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
        rightRackDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
        leftRackVoltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        rightRackVoltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        intakeDutyCycleOut = new DutyCycleOut(0.0).withEnableFOC(true);
        m_request = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(true);
        m_voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rightPosition, rightVelocity, rightVoltage, rightTorqueCurrent, leftPosition, leftVelocity, leftVoltage, leftTorqueCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            leftCurrent, leftTemp, rightCurrent, rightTemp,
            intakePosition, intakeVelocity, intakeVoltage, intakeCurrent, intakeTorqueCurrent, intakeTemp);
        PhoenixUtil.registerSignals(
            bus,
            leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp,
            rightPosition, rightVelocity, rightVoltage, rightCurrent, rightTorqueCurrent, rightTemp,
            intakePosition, intakeVelocity, intakeVoltage, intakeCurrent, intakeTorqueCurrent, intakeTemp);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftConnected =
            BaseStatusSignal.isAllGood(leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp);
        inputs.leftPositionMeters = leftPosition.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.leftVelocityMetersPerSec = leftVelocity.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftSupplyCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected =
            BaseStatusSignal.isAllGood(rightPosition, rightVelocity, rightVoltage, rightCurrent, rightTorqueCurrent, rightTemp);
        inputs.rightPositionMeters = rightPosition.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.rightVelocityMetersPerSec = rightVelocity.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();
        inputs.rightSupplyCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
        inputs.rightTempCelsius = rightTemp.getValueAsDouble();

        inputs.intakeConnected =
            BaseStatusSignal.isAllGood(intakePosition, intakeVelocity, intakeVoltage, intakeCurrent, intakeTorqueCurrent, intakeTemp);
        inputs.intakePositionRad = Rotation2d.fromRotations(intakePosition.getValueAsDouble());
        inputs.intakeVelocityRadPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
        inputs.intakeAppliedVolts = intakeVoltage.getValueAsDouble();
        inputs.intakeSupplyCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.intakeTorqueCurrentAmps = intakeTorqueCurrent.getValueAsDouble();
        inputs.intakeTempCelsius = intakeTemp.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            leftConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            rightConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));
            tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig));
        }).start();
    }

    @Override
    public void runRackPositionBoth(double positionMeters, double leftFF, double rightFF) {
        double rotations = positionMeters / IntakeConstants.METERS_PER_ROTATION;
//        leftMotor.setControl(
//            leftPositionTorqueCurrentFOC
//                .withPosition(rotations)
//                .withFeedForward(leftFF));
//        rightMotor.setControl(
//            rightPositionTorqueCurrentFOC
//                .withPosition(rotations)
//                .withFeedForward(rightFF));
        leftMotor.setControl(
                m_request.withPosition(rotations).withFeedForward(leftFF)
        );
        rightMotor.setControl(
                m_request.withPosition(rotations).withFeedForward(rightFF)
        );
    }

    @Override
    public void runRackOpenLoop(double output, boolean isTorqueCurrent) {
        leftMotor.setControl(
            isTorqueCurrent
                ? leftRackTorqueCurrentFOC.withOutput(output)
                : leftRackVoltageOut.withOutput(output));
        rightMotor.setControl(
            isTorqueCurrent
                ? rightRackTorqueCurrentFOC.withOutput(output)
                : rightRackVoltageOut.withOutput(output));
    }

    @Override
    public void runRackOpenLoopBoth(double leftOutput, double rightOutput, boolean isTorqueCurrent) {
        leftMotor.setControl(
            isTorqueCurrent
                ? leftRackTorqueCurrentFOC.withOutput(leftOutput)
                : leftRackVoltageOut.withOutput(leftOutput));
        rightMotor.setControl(
            isTorqueCurrent
                ? rightRackTorqueCurrentFOC.withOutput(rightOutput)
                : rightRackVoltageOut.withOutput(rightOutput));
    }
//
//    @Override
//    public void runRackPercentOut(double output) {
//        leftMotor.setControl(leftRackDutyCycleOut.withOutput(output));
//        rightMotor.setControl(rightRackDutyCycleOut.withOutput(output));
//    }

    @Override
    public void setRackPID(double kP, double kI, double kD) {
        leftConfig.Slot0.kP = kP;
        leftConfig.Slot0.kI = kI;
        leftConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));

        rightConfig.Slot0.kP = kP;
        rightConfig.Slot0.kI = kI;
        rightConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig));
    }

//    @Override
//    public void setPosition(double meters) {
//        new Thread(() -> {
//            tryUntilOk(5, () -> leftMotor.setPosition(meters / IntakeConstants.METERS_PER_ROTATION));
//            tryUntilOk(5, () -> rightMotor.setPosition(meters / IntakeConstants.METERS_PER_ROTATION));
//        }).start();
//    }

    @Override
    public void setPosition(double meters) {
        new Thread(() -> {
            tryUntilOk(5, () -> leftMotor.setControl(m_request.withPosition(meters)));
            tryUntilOk(5, () -> rightMotor.setControl(m_request.withPosition(meters)));

        });
    }


    @Override
    public void setMotionMagicConstraints(double cruiseVelocity, double acceleration, double jerk) {
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = cruiseVelocity / IntakeConstants.METERS_PER_ROTATION;
        mm.MotionMagicAcceleration = acceleration / IntakeConstants.METERS_PER_ROTATION;
        mm.MotionMagicJerk = jerk / IntakeConstants.METERS_PER_ROTATION;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(mm));
        tryUntilOk(5, () -> rightMotor.getConfigurator().apply(mm));
    }
//
//    @Override
//    public void runIntake(double output) {
//        intakeMotor.setControl(intakeDutyCycleOut.withOutput(output));
//    }

    @Override
    public void runIntake(double output) {
        intakeMotor.setControl(m_voltageOut.withOutput(output));
    }

    @Override
    public void runAtSysIdVoltage(double volts) {
        leftMotor.setControl(m_voltageOut.withOutput(volts));
        rightMotor.setControl(m_voltageOut.withOutput(volts));
    }


    @Override
    public void stopRack() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }
}
