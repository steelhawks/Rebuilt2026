package org.steelhawks.subsystems.superstructure.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
import org.steelhawks.subsystems.superstructure.ShooterConstants;
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
    private final TalonFX leftMotor, rightMotor;

    public FlywheelIOTalonFX(CANBus bus) {
        leftMotor = new TalonFX(ShooterConstants.Flywheel.LEFT_FLYWHEEL_ID, bus);
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.REDUCTION;
        config.Slot0.kP = ShooterConstants.Flywheel.kP.getAsDouble();
        config.Slot0.kI = ShooterConstants.Flywheel.kI.getAsDouble();
        config.Slot0.kD = ShooterConstants.Flywheel.kD.getAsDouble();

        position = leftMotor.getPosition();
        velocity = leftMotor.getVelocity();
        voltage = leftMotor.getMotorVoltage();
        current = leftMotor.getSupplyCurrent();
        torqueCurrent = leftMotor.getTorqueCurrent();
        temp = leftMotor.getDeviceTemp();

        rightMotor = new TalonFX(ShooterConstants.Flywheel.RIGHT_FLYWHEEL_ID, bus);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withSlot(0);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(1);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
        BaseStatusSignal.setUpdateFrequencyForAll(
            100, position, velocity);
        PhoenixUtil.registerSignals(
            bus.isNetworkFD(),
            position, velocity, voltage, current, torqueCurrent, temp);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(position, velocity, voltage, current, torqueCurrent, temp);
        inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.supplyCurrentAmps = current.getValueAsDouble();
        inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void runFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {
        setpoint = Units.radiansToRotations(setpoint);
        leftMotor.setControl(
            isTorqueCurrent
                ? velocityTorqueCurrentFOC.withVelocity(setpoint)
                    .withFeedForward(feedforward)
                : velocityVoltage.withVelocity(setpoint)
                    .withFeedForward(feedforward)
        );
    }

    @Override
    public void runFlywheelOpenLoop(double output, boolean isTorqueCurrent) {
        leftMotor.setControl(
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
        PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
    }
}
