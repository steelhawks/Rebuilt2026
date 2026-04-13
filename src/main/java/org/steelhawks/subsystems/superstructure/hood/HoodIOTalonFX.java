package org.steelhawks.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
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

public class HoodIOTalonFX implements HoodIO {

    private final TalonFX hoodMotor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> deviceTemp;

    private final TalonFXConfiguration motorConfig;

    private final TorqueCurrentFOC torqueCurrentFOC;
    private final MotionMagicVoltage motionMagicVoltage;
    private final VoltageOut voltageOut;

    public HoodIOTalonFX(CANBus bus, SubsystemConstants.HoodConstants constants) {
        hoodMotor = new TalonFX(constants.motorId(), bus);

        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        motionMagicVoltage = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

        motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.Slot0.kP = constants.kP();
        motorConfig.Slot0.kI = constants.kI();
        motorConfig.Slot0.kD = constants.kD();
        motorConfig.Slot0.kS = constants.kS();
        motorConfig.Slot0.kG = constants.kG();
        motorConfig.Slot0.kA = constants.kA();

        motorConfig.Feedback.SensorToMechanismRatio = constants.reduction();

        motorConfig.CurrentLimits.SupplyCurrentLimit = CurrentLimits.SupplyLimit.hoodCurrent;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.SupplyLimit.hoodEnabled;
        motorConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.StatorLimit.hoodCurrent;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = CurrentLimits.StatorLimit.hoodEnabled;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = constants.maxVelocity();
        motorConfig.MotionMagic.MotionMagicAcceleration = constants.maxAcceleration();

        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig));

        position = hoodMotor.getPosition();
        velocity = hoodMotor.getVelocity();
        appliedVolts = hoodMotor.getMotorVoltage();
        supplyCurrent = hoodMotor.getSupplyCurrent();
        torqueCurrent = hoodMotor.getTorqueCurrent();
        deviceTemp = hoodMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            position,
            velocity);

        PhoenixUtil.registerSignals(
            bus,
            position, velocity, appliedVolts, supplyCurrent, torqueCurrent, deviceTemp);
        tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(hoodMotor));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected =
            BaseStatusSignal.isAllGood(position, velocity, appliedVolts, supplyCurrent, torqueCurrent, deviceTemp);
        inputs.motorPositionDeg = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.motorVelocityDegPerSec = Units.rotationsToDegrees(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = deviceTemp.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig));
        }).start();
    }

    @Override
    public void runHoodPosition(Rotation2d setpoint, double feedforward) {
        hoodMotor.setControl(
            motionMagicVoltage
                .withPosition(setpoint.getRotations())
                .withFeedForward(feedforward));
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        hoodMotor.setControl(
            isTorqueCurrent
                ? torqueCurrentFOC.withOutput(output)
                : voltageOut.withOutput(output));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig));
    }

    @Override
    public void setPosition(Rotation2d position) {
        tryUntilOk(5, () -> hoodMotor.setPosition(position.getRotations()));
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }
}