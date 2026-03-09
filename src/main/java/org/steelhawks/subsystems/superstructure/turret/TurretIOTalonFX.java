package org.steelhawks.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;

    private final PositionVoltage positionVoltage;
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final DutyCycleOut dutyCycleOut;

    private final TalonFXConfiguration config;
    private final TalonFX motor;

    public TurretIOTalonFX(CANBus bus, SubsystemConstants.TurretConstants constants) {
        motor = new TalonFX(constants.turretId(), bus);
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kP = constants.kP();
        config.Slot0.kI = constants.kI();
        config.Slot0.kD = constants.kD();
        config.Feedback.SensorToMechanismRatio = constants.motorReduction();
        config.ClosedLoopGeneral.ContinuousWrap = false;
        config.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimitAmps();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, motor::optimizeBusUtilization);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        current = motor.getSupplyCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temp = motor.getDeviceTemp();

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withSlot(0);
        positionVoltage = new PositionVoltage(0.0).withSlot(1);
        voltageOut = new VoltageOut(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0);
        dutyCycleOut = new DutyCycleOut(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            100, position, velocity, voltage, current, torqueCurrent, temp);
        PhoenixUtil.registerSignals(
            bus,
            position, velocity, voltage, current, torqueCurrent, temp);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, voltage, current, torqueCurrent, temp);
        inputs.positionRad = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.velocityRadPerSec = Rotation2d.fromRotations(velocity.getValueAsDouble());
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrentAmps = current.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        new Thread(() -> {
            config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        }).start();
    }

    @Override
    public void runPivot(double setpoint, double feedforward) {
        motor.setControl(
            positionTorqueCurrentFOC.withPosition(Units.radiansToRotations(setpoint))
                .withFeedForward(feedforward));
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        motor.setControl(
            isTorqueCurrent
                ? torqueCurrentFOC.withOutput(output)
                : voltageOut.withOutput(output));
    }

    @Override
    public void runPercentOutput(double output) {
        motor.setControl(
            dutyCycleOut.withOutput(output));
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        config.Slot0.kP = kp;
        config.Slot0.kI = ki;
        config.Slot0.kD = kd;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    @Override
    public void setPosition(double position) {
        new Thread(() -> {
            motor.setPosition(Units.radiansToRotations(position));
        }).start();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
