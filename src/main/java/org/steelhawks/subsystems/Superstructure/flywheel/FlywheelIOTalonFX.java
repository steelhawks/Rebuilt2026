package org.steelhawks.subsystems.Superstructure.flywheel;

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
import org.steelhawks.util.PhoenixUtil;


public class FlywheelIOTalonFX implements FlywheelIO {
    private final TalonFX left_motor;
    private final TalonFX right_motor;

    private final TalonFXConfiguration config;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocityRadPerSec;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> tempCelcius;

    private final VoltageOut voltageOut;
    private final VelocityVoltage velocityVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public FlywheelIOTalonFX(CANBus bus, FlywheelConstants constants) {
        left_motor = new TalonFX( constants.leftMotorID, bus);
        right_motor = new TalonFX(constants.rightMotorID, bus);

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = constants.GEAR_RATIO;

        config.Slot0.kP = constants.kP();
        config.Slot0.kI = constants.kI();
        config.Slot0.kD = constants.kD();

        right_motor.setControl(new Follower(constants.leftMotorID, MotorAlignmentValue.Opposed));

        PhoenixUtil.tryUntilOk(5, () -> left_motor.getConfigurator().apply(config));

        position = left_motor.getPosition();
        velocityRadPerSec = left_motor.getVelocity();
        currentAmps = left_motor.getSupplyCurrent();
        appliedVolts = left_motor.getMotorVoltage();
        torqueCurrent = left_motor.getTorqueCurrent();
        statorCurrent = left_motor.getStatorCurrent();
        tempCelcius = left_motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(1000, position, velocityRadPerSec);

        PhoenixUtil.registerSignals(
                bus,
                position,
                velocityRadPerSec,
                currentAmps,
                appliedVolts,
                torqueCurrent,
                statorCurrent,
                torqueCurrent
        );

        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);


    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.isAllGood( position,
                velocityRadPerSec,
                currentAmps,
                appliedVolts,
                torqueCurrent,
                statorCurrent,
                torqueCurrent
        );
        inputs.position = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRadPerSec.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
        inputs.tempCelsius = tempCelcius.getValueAsDouble();

    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        left_motor.setControl(
                isTorqueCurrent ? torqueCurrentFOC.withOutput(output) : voltageOut.withOutput(output)
        );
    }

    @Override
    public void runFlywheel(double position, double output, boolean isTorqueCurrent) {
        position = Units.radiansToRotations(position);

        left_motor.setControl(
                isTorqueCurrent ? velocityTorqueCurrentFOC.withVelocity(position).withFeedForward(output) :
                        velocityVoltage.withVelocity(position).withFeedForward(output)
        );
    }

    @Override
    public void setFlywheelPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> left_motor.getConfigurator().apply(config));
    }

    @Override
    public void stopFlywheel() {
        left_motor.stopMotor();
    }

}
