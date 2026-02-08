package org.steelhawks.subsystems.spindexer;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;

public class SpindexerIOTalonFX implements SpindexerIO {

	private final StatusSignal<Angle> position;
	private final StatusSignal<AngularVelocity> velocity;
	private final StatusSignal<Voltage> voltage;
	private final StatusSignal<Current> current;
	private final StatusSignal<Current> torqueCurrent;
	private final StatusSignal<Temperature> temp;

	private final int MOTOR_ID = 0;
	private final TalonFX motor;
	private final TalonFXConfiguration config;

	private final DutyCycleOut dutyCycleOut;

	public SpindexerIOTalonFX(RobotConfig.CANBus canBus) {
		motor = new TalonFX(MOTOR_ID, canBus.bus);

		config = new TalonFXConfiguration();
		config.Feedback.SensorToMechanismRatio = 15.0 / 1.0;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
		PhoenixUtil.tryUntilOk(5, motor::optimizeBusUtilization);

		position = motor.getPosition();
		velocity = motor.getVelocity();
		voltage = motor.getMotorVoltage();
		current = motor.getStatorCurrent();
		torqueCurrent = motor.getTorqueCurrent();
		temp = motor.getDeviceTemp();

		dutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
		BaseStatusSignal.setUpdateFrequencyForAll(
		50,
			velocity,
			voltage,
			current,
			torqueCurrent);
		PhoenixUtil.registerSignals(canBus.bus.isNetworkFD(),
			position,
			velocity,
			voltage,
			current,
			torqueCurrent,
			temp);
	}

	@Override
	public void updateInputs(SpindexerIOInputs inputs) {
		inputs.connected = BaseStatusSignal.isAllGood(
			position, velocity, voltage, current, torqueCurrent, temp);
		inputs.positionRad = position.getValueAsDouble();
		inputs.velocityRadPerSec = velocity.getValueAsDouble();
		inputs.appliedVolts = voltage.getValueAsDouble();
		inputs.currentAmps = current.getValueAsDouble();
		inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
		inputs.tempCelsius = temp.getValueAsDouble();
	}

	@Override
	public void setBrakeMode(boolean enabled) {
		new Thread(
			() -> {
				config.MotorOutput.NeutralMode =
					enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
				PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
		}).start();
	}

	@Override
	public void runSpindexer(double output) {
		motor.setControl(
			dutyCycleOut.withOutput(output));
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}
}
