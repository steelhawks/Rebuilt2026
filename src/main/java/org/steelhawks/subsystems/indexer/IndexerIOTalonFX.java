package org.steelhawks.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

	private final StatusSignal<Angle> spindexer1Position;
	private final StatusSignal<AngularVelocity> spindexer1Velocity;
	private final StatusSignal<Voltage> spindexer1Voltage;
	private final StatusSignal<Current> spindexer1Current;
	private final StatusSignal<Current> spindexer1TorqueCurrent;
	private final StatusSignal<Temperature> spindexer1Temp;

    private final StatusSignal<Angle> feederPosition;
    private final StatusSignal<AngularVelocity> feederVelocity;
    private final StatusSignal<Voltage> feederVoltage;
    private final StatusSignal<Current> feederCurrent;
    private final StatusSignal<Current> feederTorqueCurrent;
    private final StatusSignal<Temperature> feederTemp;

	private final TalonFX spindexerMotor;
    private final TalonFX feederMotor;
	private final TalonFXConfiguration spindexerConfig;
    private final TalonFXConfiguration feederConfig;

	private final DutyCycleOut spindexerDutyCycleOut;
    private final DutyCycleOut feederDutyCycleOut;

	// optional spindexer motor
	// these are explicitly null if an motor id isn't present for the 2nd motor in the constants record, please null check before accessing
	private TalonFX spindexerMotor2 = null;
	private StatusSignal<Angle> spindexer2Position = null;
	private StatusSignal<AngularVelocity> spindexer2Velocity = null;
	private StatusSignal<Voltage> spindexer2Voltage = null;
	private StatusSignal<Current> spindexer2Current = null;
	private StatusSignal<Current> spindexer2TorqueCurrent = null;
	private StatusSignal<Temperature> spindexer2Temperature = null;

	public IndexerIOTalonFX(CANBus canBus, SubsystemConstants.IndexerConstants constants) {
        spindexerMotor = new TalonFX(constants.spindexerMotor1Id(), canBus);
        feederMotor = new TalonFX(constants.feederId(), canBus);

		spindexerConfig = new TalonFXConfiguration();
		spindexerConfig.Feedback.SensorToMechanismRatio = 15.0 / 1.0;
		spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		spindexerConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
		spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(spindexerConfig));
		PhoenixUtil.tryUntilOk(5, spindexerMotor::optimizeBusUtilization);

        feederConfig = new TalonFXConfiguration();
        feederConfig.Feedback.SensorToMechanismRatio = 1.0;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		feederConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig));
        PhoenixUtil.tryUntilOk(5, feederMotor::optimizeBusUtilization);

		spindexer1Position = spindexerMotor.getPosition();
        spindexer1Velocity = spindexerMotor.getVelocity();
		spindexer1Voltage = spindexerMotor.getMotorVoltage();
		spindexer1Current = spindexerMotor.getStatorCurrent();
		spindexer1TorqueCurrent = spindexerMotor.getTorqueCurrent();
		spindexer1Temp = spindexerMotor.getDeviceTemp();

        feederPosition = feederMotor.getPosition();
        feederVelocity = feederMotor.getVelocity();
        feederVoltage = feederMotor.getMotorVoltage();
        feederCurrent = feederMotor.getSupplyCurrent();
        feederTorqueCurrent = feederMotor.getTorqueCurrent();
        feederTemp = feederMotor.getDeviceTemp();

		if (constants.spindexerMotor2Id().isPresent()) {
			spindexerMotor2 = new TalonFX(constants.spindexerMotor2Id().getAsInt(), canBus);
			spindexerMotor2.setControl(new Follower(spindexerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
			PhoenixUtil.tryUntilOk(5, () -> spindexerMotor2.getConfigurator().apply(spindexerConfig));
			PhoenixUtil.tryUntilOk(5, spindexerMotor2::optimizeBusUtilization);

			spindexer2Position = spindexerMotor2.getPosition();
			spindexer2Velocity = spindexerMotor2.getVelocity();
			spindexer2Voltage = spindexerMotor2.getMotorVoltage();
			spindexer2Current = spindexerMotor2.getStatorCurrent();
			spindexer2TorqueCurrent = spindexerMotor2.getTorqueCurrent();
			spindexer2Temperature = spindexerMotor2.getDeviceTemp();

			BaseStatusSignal.setUpdateFrequencyForAll(
				50,
				spindexer2Velocity,
				spindexer2Voltage,
				spindexer2Current,
				spindexer2TorqueCurrent
			);

			PhoenixUtil.registerSignals(
				canBus,
				spindexer2Position,
				spindexer2Velocity,
				spindexer2Voltage,
				spindexer2Current,
				spindexer2TorqueCurrent,
				spindexer2Temperature
			);
		}

		spindexerDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
		feederDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
		BaseStatusSignal.setUpdateFrequencyForAll(
		50,
			spindexer1Velocity,
			spindexer1Voltage,
			spindexer1Current,
			spindexer1TorqueCurrent,
			feederVelocity,
			feederVoltage,
			feederCurrent,
			feederTorqueCurrent);
		PhoenixUtil.registerSignals(canBus,
			spindexer1Position,
			spindexer1Velocity,
			spindexer1Voltage,
			spindexer1Current,
			spindexer1TorqueCurrent,
			spindexer1Temp,
			feederPosition,
			feederVelocity,
			feederVoltage,
			feederCurrent,
			feederTorqueCurrent,
			feederTemp);
	}

	@Override
	public void updateInputs(SpindexerIOInputs spindexerInputs, FeederIOInputs feederInputs) {
		spindexerInputs.motor1Connected = BaseStatusSignal.isAllGood(
			spindexer1Position, spindexer1Velocity, spindexer1Voltage, spindexer1Current, spindexer1TorqueCurrent, spindexer1Temp);
		spindexerInputs.motor1PositionRad = spindexer1Position.getValueAsDouble();
		spindexerInputs.motor1VelocityRadPerSec = spindexer1Velocity.getValueAsDouble();
		spindexerInputs.motor1AppliedVolts = spindexer1Voltage.getValueAsDouble();
		spindexerInputs.motor1CurrentAmps = spindexer1Current.getValueAsDouble();
		spindexerInputs.motor1TorqueCurrentAmps = spindexer1TorqueCurrent.getValueAsDouble();
		spindexerInputs.motor1TempCelsius = spindexer1Temp.getValueAsDouble();

        feederInputs.connected = BaseStatusSignal.isAllGood(
            feederPosition, feederVelocity, feederVoltage, feederCurrent, feederTorqueCurrent, feederTemp);
        feederInputs.positionRad = feederPosition.getValueAsDouble();
        feederInputs.velocityRadPerSec = feederVelocity.getValueAsDouble();
        feederInputs.appliedVolts = feederVoltage.getValueAsDouble();
        feederInputs.currentAmps = feederCurrent.getValueAsDouble();
        feederInputs.torqueCurrentAmps = feederTorqueCurrent.getValueAsDouble();
        feederInputs.tempCelsius = feederTemp.getValueAsDouble();

		if (spindexerMotor2 != null) {
			spindexerInputs.motor2Connected = BaseStatusSignal.isAllGood(
				spindexer2Position, spindexer2Velocity, spindexer2Voltage, spindexer2Current, spindexer2TorqueCurrent, spindexer2Temperature);
			spindexerInputs.motor2PositionRad = spindexer2Position.getValueAsDouble();
			spindexerInputs.motor2VelocityRadPerSec = spindexer2Velocity.getValueAsDouble();
			spindexerInputs.motor2AppliedVolts = spindexer2Voltage.getValueAsDouble();
			spindexerInputs.motor2CurrentAmps = spindexer2Current.getValueAsDouble();
			spindexerInputs.motor2TorqueCurrentAmps = spindexer2TorqueCurrent.getValueAsDouble();
			spindexerInputs.motor2TempCelsius = spindexer2Temperature.getValueAsDouble();
		} else {
			spindexerInputs.motor2Connected = false;
		}
	}

	@Override
	public void setBrakeMode(boolean enabled) {
		new Thread(
			() -> {
				spindexerConfig.MotorOutput.NeutralMode =
					enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
				PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(spindexerConfig));
                feederConfig.MotorOutput.NeutralMode =
                    enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig));
		}).start();
	}

	@Override
	public void runSpindexer(double output) {
		spindexerMotor.setControl(
			spindexerDutyCycleOut.withOutput(output));
	}

    @Override
    public void runFeeder(double output) {
        feederMotor.setControl(
            feederDutyCycleOut.withOutput(output));
    }

	@Override
	public void stopSpindexer() {
		spindexerMotor.stopMotor();
	}

    @Override
    public void stopFeeder() {
        feederMotor.stopMotor();
    }
}
