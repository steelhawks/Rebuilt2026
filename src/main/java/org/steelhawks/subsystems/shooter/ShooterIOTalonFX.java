package org.steelhawks.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.steelhawks.Constants;
import org.steelhawks.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
    public final TalonFX flywheelMotor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> amperage;
    private final StatusSignal<Temperature> temp;

    private final VelocityVoltage velocityOut;
    private final VoltageOut voltageOut;

    public ShooterIOTalonFX() {
        flywheelMotor = new TalonFX(0, Constants.getCANBus());

        config.Slot0.kP = ShooterConstants.kP.getAsDouble();
        config.Slot0.kI = ShooterConstants.kI.getAsDouble();
        config.Slot0.kD = ShooterConstants.kD.getAsDouble();

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(config));

        velocity = flywheelMotor.getVelocity();
        voltage = flywheelMotor.getMotorVoltage();
        temp = flywheelMotor.getDeviceTemp();
        amperage = flywheelMotor.getStatorCurrent();

        PhoenixUtil.registerSignals(false,
                velocity,
                voltage,
                temp,
                amperage
        );

        flywheelMotor.optimizeBusUtilization();

        velocityOut = new VelocityVoltage(0.0);
        voltageOut = new VoltageOut(0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(voltage, velocity, temp, amperage);
        inputs.velocityRadPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
        inputs.currentAmps = amperage.getValueAsDouble();
    }

    @Override
    public void runOpenLoop(double volts) {
        flywheelMotor.setControl(
                voltageOut.withOutput(volts)
        );
    }

    @Override
    public void runVelocity(double rps, double feedforward) {
        flywheelMotor.setControl(
                velocityOut.withVelocity(rps)
                        .withFeedForward(feedforward)
        );
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(config));
    }
}
