package org.steelhawks.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
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

    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final TorqueCurrentFOC rackTorqueCurrentFOC;
    private final DutyCycleOut rackDutyCycleOut;
    private final VoltageOut rackVoltageOut;

    private final DutyCycleOut intakeDutyCycleOut;

    private final TalonFXConfiguration leftConfig;
    private final TalonFXConfiguration intakeConfig;

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX intakeMotor;

    public IntakeIOTalonFX(RobotConfig.CANBus bus) {
        leftMotor = new TalonFX(IntakeConstants.LEFT_ID, bus.bus);
        rightMotor = new TalonFX(IntakeConstants.RIGHT_ID, bus.bus);
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_ID, bus.bus);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        leftConfig = new TalonFXConfiguration();
        intakeConfig = new TalonFXConfiguration();

        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.Slot0.kP = IntakeConstants.kP.getAsDouble();
        leftConfig.Slot0.kI = IntakeConstants.kI.getAsDouble();
        leftConfig.Slot0.kD = IntakeConstants.kD.getAsDouble();
        leftConfig.Feedback.SensorToMechanismRatio = IntakeConstants.REDUCTION;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig));
        tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor, intakeMotor));

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

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        rackTorqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        rackDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
        rackVoltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        intakeDutyCycleOut = new DutyCycleOut(0.0).withEnableFOC(true);

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            leftPosition,
            leftVelocity,
            rightPosition,
            rightVelocity,
            intakePosition,
            intakeVelocity);
        PhoenixUtil.registerSignals(
            bus.bus.isNetworkFD(),
            leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp,
            rightPosition, rightVelocity, rightVelocity, rightCurrent, rightTorqueCurrent, rightTemp,
            intakePosition, intakeVelocity, intakeCurrent, intakeTorqueCurrent, intakeTemp
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftConnected =
            BaseStatusSignal.isAllGood(leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTorqueCurrent, leftTemp);
        inputs.leftPositionMeters = leftPosition.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.leftVelocityMetersPerSec = leftVelocity.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
        inputs.leftTempCelsius = leftTemp.getValueAsDouble();

        inputs.rightConnected =
            BaseStatusSignal.isAllGood(rightPosition, rightVelocity, rightVelocity, rightCurrent, rightTorqueCurrent, rightTemp);
        inputs.rightPositionMeters = rightPosition.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
        inputs.rightVelocityMetersPerSec = rightVelocity.getValueAsDouble() * IntakeConstants.METERS_PER_ROTATION;
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
    public void runRackPosition(double positionMeters, double feedforward) {
        leftMotor.setControl(
            positionTorqueCurrentFOC
                .withPosition(positionMeters / IntakeConstants.METERS_PER_ROTATION)
                .withFeedForward(feedforward));
    }

    @Override
    public void runRackOpenLoop(double output, boolean isTorqueCurrent) {
        leftMotor.setControl(
            isTorqueCurrent
                ? rackTorqueCurrentFOC.withOutput(output)
                : rackVoltageOut.withOutput(output));
    }

    @Override
    public void runRackPercentOut(double output) {
        leftMotor.setControl(
            rackDutyCycleOut.withOutput(output));
    }

    @Override
    public void setRackPID(double kP, double kI, double kD) {
        leftConfig.Slot0.kP = kP;
        leftConfig.Slot0.kI = kI;
        leftConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));
    }

    @Override
    public void setPosition(double meters) {
        new Thread(() -> tryUntilOk(5, () -> leftMotor.setPosition(meters / IntakeConstants.METERS_PER_ROTATION)));
    }

    @Override
    public void runIntake(double output) {
        intakeMotor.setControl(
            intakeDutyCycleOut.withOutput(output));
    }

    @Override
    public void stopRack() {
        leftMotor.stopMotor();
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }
}
