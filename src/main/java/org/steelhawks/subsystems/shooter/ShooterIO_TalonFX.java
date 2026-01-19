package org.steelhawks.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DutyCycle;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class ShooterIO_TalonFX implements ShooterIO {
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final VoltageOut voltageOut;
    private final DutyCycleOut dutyCycleOut;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;

    public ShooterIO_TalonFX() {
        masterMotor = new TalonFX(ShooterConstants.SHOOTER_MASTERMOTOR_ID_OMEGA, Constants.getCANBus());
        followerMotor = new TalonFX(ShooterConstants.SHOOTER_FOLLOWERMOTOR_ID_OMEGA, Constants.getCANBus());

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = new Slot0Configs()
            .withKP(ShooterConstants.kP.get())
            .withKI(ShooterConstants.kI.get())
            .withKD(ShooterConstants.kD.get());
        masterMotor.getConfigurator().apply(config.Slot0);
        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_REDUCTION; //gear ratio conversion
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> masterMotor.getConfigurator().apply(config, 0.25));

        followerMotor.setControl(
            new StrictFollower(masterMotor.getDeviceID()));

        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
        dutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);

        position = masterMotor.getPosition();
        velocity = masterMotor.getVelocity();
        voltage = masterMotor.getMotorVoltage();
        current = masterMotor.getStatorCurrent();
        temp = masterMotor.getDeviceTemp();

        //updates signal data every 10ms
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            position,
            velocity,
            voltage,
            current,
            temp
        );

        masterMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterMotorConnected =
            BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                current,
                temp
            ).isOK();
        inputs.masterMotorPositionRad = Units.rotationsToRadians(position.getValueAsDouble());
        inputs.masterMotorVelocityRPM = Units.rotationsToRadians(velocity.getValueAsDouble());
        inputs.masterMotorAppliedVolts = voltage.getValueAsDouble();
        inputs.masterMotorCurrentAmps = current.getValueAsDouble();
        inputs.masterMotorTemperatureCelsius = temp.getValueAsDouble();
    }

    @Override
    public void runOpenLoop(double output) {
        masterMotor.setControl(
            voltageOut.withOutput(output)
        );
    }
    @Override
    public void runSpeed(double speed) {
        masterMotor.setControl(
            dutyCycleOut.withOutput(speed)
        );
    }
    @Override
    public void zeroEncoders() {
        masterMotor.setPosition(0.0);
    }
    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        tryUntilOk(5, () -> masterMotor.getConfigurator().apply(config));
    }
    @Override
    public void setBrakeMode(boolean enabled) {
        masterMotor.setNeutralMode(
            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast
        );
    }
    @Override
    public void stop() {
        masterMotor.stopMotor();
    }
}
