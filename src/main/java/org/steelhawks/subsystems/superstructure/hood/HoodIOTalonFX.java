package org.steelhawks.subsystems.superstructure.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class HoodIOTalonFX implements HoodIO {

    private final TalonFX hoodMotor;
    private final CANcoder cancoder;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> deviceTemp;

    private final StatusSignal<Angle> cancoderPosition;
    private final StatusSignal<AngularVelocity> cancoderVelocity;
    private final StatusSignal<Voltage> cancoderVoltage;

    private final TalonFXConfiguration motorConfig;
    private final CANcoderConfiguration cancoderConfig;

    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    public HoodIOTalonFX(RobotConfig.CANBus bus) {
        hoodMotor = new TalonFX(HoodConstants.MOTOR_ID, bus.bus);
        cancoder = new CANcoder(HoodConstants.CANCODER_ID);

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

        motorConfig = new TalonFXConfiguration();
        cancoderConfig = new CANcoderConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.Slot0.kP = HoodConstants.kP.get();
        motorConfig.Slot0.kI = HoodConstants.kI.get();
        motorConfig.Slot0.kD = HoodConstants.kD.get();
        motorConfig.Feedback.SensorToMechanismRatio = HoodConstants.REDUCTION;
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig));

        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = HoodConstants.MAG_OFFSET.getRotations();
        tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

        position = hoodMotor.getPosition();
        velocity = hoodMotor.getVelocity();
        appliedVolts = hoodMotor.getMotorVoltage();
        statorCurrent = hoodMotor.getStatorCurrent();
        torqueCurrent = hoodMotor.getTorqueCurrent();
        deviceTemp = hoodMotor.getDeviceTemp();

        cancoderPosition = cancoder.getPosition();
        cancoderVelocity = cancoder.getVelocity();
        cancoderVoltage = cancoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            position,
            velocity,
            cancoderPosition,
            cancoderVelocity,
            cancoderVoltage);

        PhoenixUtil.registerSignals(
            bus.bus.isNetworkFD(),
            position, velocity, appliedVolts, statorCurrent, torqueCurrent, deviceTemp);
        tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(hoodMotor, cancoder));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected =
            BaseStatusSignal.isAllGood(position, velocity, appliedVolts, statorCurrent, torqueCurrent, deviceTemp);
        inputs.motorPositionDeg = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.motorVelocityDegPerSec = Units.rotationsToDegrees(velocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = statorCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = deviceTemp.getValueAsDouble();

        inputs.cancoderConnected =
            BaseStatusSignal.isAllGood(cancoderPosition, cancoderVelocity, cancoderVoltage);
        inputs.cancoderPositionDeg = Rotation2d.fromRotations(cancoderPosition.getValueAsDouble());
        inputs.cancoderVelocityDegPerSec = Units.rotationsToDegrees(cancoderVelocity.getValueAsDouble());
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
            positionTorqueCurrentFOC
                .withPosition(setpoint.getRotations())
                .withFeedForward(feedforward));
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        hoodMotor.setControl(
            isTorqueCurrent
                ? torqueCurrentFOC.withOutput(output)
                : voltageOut.withOutput(output)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig));
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }
}
