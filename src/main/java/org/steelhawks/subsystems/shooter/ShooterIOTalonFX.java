package org.steelhawks.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.Serializers;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.Constants;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class ShooterIOTalonFX implements ShooterIO {

	private final TalonFXConfiguration config = new TalonFXConfiguration();

	private final TalonFX motor;

	private final VoltageOut voltageOut;
	private final DutyCycleOut dutyCycle;

	private final StatusSignal<Angle> position;
	private final StatusSignal<AngularVelocity> velocity;
	private final StatusSignal<Voltage> voltage;
	private final StatusSignal<Current> current;
	private final StatusSignal<Temperature> temp;

	public ShooterIOTalonFX() {

		motor = new TalonFX(ShooterConstants.MOTOR_ID, Constants.getCANBus());

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.Slot0 = new Slot0Configs()
			.withKP(ShooterConstants.KP.get())
			.withKI(ShooterConstants.KI)
			.withKD(ShooterConstants.KD);
		config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_REDUCTION;
		config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

		voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
		dutyCycle = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);

		position = motor.getPosition();
		velocity = motor.getVelocity();
		voltage = motor.getSupplyVoltage();
		current = motor.getStatorCurrent();
		temp = motor.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(
			100,
			position,
			velocity,
			voltage,
			current,
			temp);

		motor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.connected =
			BaseStatusSignal.refreshAll(
				position,
				velocity,
				voltage,
				current,
				temp).isOK();
		inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
		inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
		inputs.appliedVolts = voltage.getValueAsDouble();
		inputs.currentAmps = current.getValueAsDouble();
		inputs.tempCelsius = temp.getValueAsDouble();
	}

	@Override
	public void runOpenLoop(double output) {
		motor.setControl(
			voltageOut.withOutput(output));
	}

	@Override
	public void runSpeed(double speed) {
		motor.setControl(
			dutyCycle.withOutput(speed));
	}

	@Override
	public void zeroEncoders() {
		motor.setPosition(0.0);
	}

	@Override
	public void setPID(double kP, double kI, double kD) {
		config.Slot0.kP = kP;
		config.Slot0.kI = kI;
		config.Slot0.kD = kD;
		tryUntilOk(5, () -> motor.getConfigurator().apply(config));
	}

	@Override
	public void setBrakeMode(boolean enabled) {
		motor.setNeutralMode(
			enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}
}

