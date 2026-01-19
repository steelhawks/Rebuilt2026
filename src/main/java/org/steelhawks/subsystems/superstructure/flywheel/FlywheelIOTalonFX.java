package org.steelhawks.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Temperature> temp;

    private final VelocityVoltage velocityVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    private final TalonFXConfiguration config;
    private final TalonFX motor;

    public FlywheelIOTalonFX(RobotConfig.CANBus bus) {
        motor = new TalonFX(Flywheel.motorId, bus.bus);
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kP = Flywheel.kP.getAsDouble();
        config.Slot0.kI = Flywheel.kI.getAsDouble();
        config.Slot0.kD = Flywheel.kD.getAsDouble();

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        current = motor.getStatorCurrent();
        torqueCurrent = motor.getTorqueCurrent();
        temp = motor.getDeviceTemp();

        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withSlot(0);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(1);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
        PhoenixUtil.registerSignals(
            bus.bus.isNetworkFD(),
            position, velocity, voltage, current, torqueCurrent, temp);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, voltage, current, torqueCurrent, temp);
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void runFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {
        motor.setControl(
            isTorqueCurrent
                ? velocityTorqueCurrentFOC.withVelocity(setpoint)
                    .withFeedForward(feedforward)
                : velocityVoltage.withVelocity(setpoint)
                    .withFeedForward(feedforward)
        );
    }

    @Override
    public void runFlywheelOpenLoop(double output, boolean isTorqueCurrent) {
        motor.setControl(
            isTorqueCurrent
                ? torqueCurrentFOC.withOutput(output)
                : voltageOut.withOutput(output)
        );
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
