package org.steelhawks.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

	private final StatusSignal<Angle> spindexerPosition;
	private final StatusSignal<AngularVelocity> spindexerVelocity;
	private final StatusSignal<Voltage> spindexerVoltage;
	private final StatusSignal<Current> spindexerCurrent;
	private final StatusSignal<Current> spindexerTorqueCurrent;
	private final StatusSignal<Temperature> spindexerTemp;

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

	public IndexerIOTalonFX(RobotConfig.CANBus canBus, SubsystemConstants.IndexerConstants constants) {
        spindexerMotor = new TalonFX(constants.spindexerMotor1Id(), canBus.bus);
        feederMotor = new TalonFX(constants.feederId(), canBus.bus);

		spindexerConfig = new TalonFXConfiguration();
		spindexerConfig.Feedback.SensorToMechanismRatio = 15.0 / 1.0;
		spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(spindexerConfig));
		PhoenixUtil.tryUntilOk(5, spindexerMotor::optimizeBusUtilization);

        feederConfig = new TalonFXConfiguration();
        feederConfig.Feedback.SensorToMechanismRatio = 1.0;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig));
        PhoenixUtil.tryUntilOk(5, feederMotor::optimizeBusUtilization);

		spindexerPosition = spindexerMotor.getPosition();
        spindexerVelocity = spindexerMotor.getVelocity();
		spindexerVoltage = spindexerMotor.getMotorVoltage();
		spindexerCurrent = spindexerMotor.getStatorCurrent();
		spindexerTorqueCurrent = spindexerMotor.getTorqueCurrent();
		spindexerTemp = spindexerMotor.getDeviceTemp();

        feederPosition = feederMotor.getPosition();
        feederVelocity = feederMotor.getVelocity();
        feederVoltage = feederMotor.getMotorVoltage();
        feederCurrent = feederMotor.getStatorCurrent();
        feederTorqueCurrent = feederMotor.getTorqueCurrent();
        feederTemp = feederMotor.getDeviceTemp();

		spindexerDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
		feederDutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
		BaseStatusSignal.setUpdateFrequencyForAll(
		50,
			spindexerVelocity,
            spindexerVoltage,
            spindexerCurrent,
            spindexerTorqueCurrent,
			feederVelocity,
			feederVoltage,
			feederCurrent,
			feederTorqueCurrent);
		PhoenixUtil.registerSignals(canBus.bus.isNetworkFD(),
            spindexerPosition,
            spindexerVelocity,
            spindexerVoltage,
            spindexerCurrent,
            spindexerTorqueCurrent,
            spindexerTemp,
			feederPosition,
			feederVelocity,
			feederVoltage,
			feederCurrent,
			feederTorqueCurrent,
			feederTemp);
	}

	@Override
	public void updateInputs(SpindexerIOInputs spindexerInputs, FeederIOInputs feederInputs) {
		spindexerInputs.connected = BaseStatusSignal.isAllGood(
            spindexerPosition, spindexerVelocity, spindexerVoltage, spindexerCurrent, spindexerTorqueCurrent, spindexerTemp);
		spindexerInputs.positionRad = spindexerPosition.getValueAsDouble();
		spindexerInputs.velocityRadPerSec = spindexerVelocity.getValueAsDouble();
		spindexerInputs.appliedVolts = spindexerVoltage.getValueAsDouble();
		spindexerInputs.currentAmps = spindexerCurrent.getValueAsDouble();
		spindexerInputs.torqueCurrentAmps = spindexerTorqueCurrent.getValueAsDouble();
		spindexerInputs.tempCelsius = spindexerTemp.getValueAsDouble();

        feederInputs.connected = BaseStatusSignal.isAllGood(
            feederPosition, feederVelocity, feederVoltage, feederCurrent, feederTorqueCurrent, feederTemp);
        feederInputs.positionRad = feederPosition.getValueAsDouble();
        feederInputs.velocityRadPerSec = feederVelocity.getValueAsDouble();
        feederInputs.appliedVolts = feederVoltage.getValueAsDouble();
        feederInputs.currentAmps = feederCurrent.getValueAsDouble();
        feederInputs.torqueCurrentAmps = feederTorqueCurrent.getValueAsDouble();
        feederInputs.tempCelsius = feederTemp.getValueAsDouble();
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
