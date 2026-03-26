package org.steelhawks.subsystems.superstructure.turret;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.steelhawks.BuilderConstants;
import org.steelhawks.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turret;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocityPerSec;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Temperature> tempCelcius;

    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<Voltage> encoderVolts;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    private final TalonFXConfiguration turretConfig;

    private final CANcoderConfiguration caNcoderConfiguration;
    private final CANcoder cancoder;

    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final DutyCycleOut dutyCycleOut;
    private final VoltageOut voltageOut;

    public TurretIOTalonFX(CANBus bus, BuilderConstants.TurretConstants constants) {
        turret = new TalonFX(constants.turretId(), bus);

        turretConfig = new TalonFXConfiguration();

        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.SupplyCurrentLimit = 20;
        turretConfig.Feedback.SensorToMechanismRatio = constants.motorReduction();
        turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretConfig.Slot0.kP = constants.kP();
        turretConfig.Slot0.kI = constants.kI();
        turretConfig.Slot0.kD = constants.kD();

        cancoder = new CANcoder(constants.encoderId(), bus);
        caNcoderConfiguration = new CANcoderConfiguration();

        caNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        caNcoderConfiguration.MagnetSensor.MagnetOffset = constants.encoderOffset().getRotations();

        PhoenixUtil.tryUntilOk(5, () -> turret.getConfigurator().apply(turretConfig));
        PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(caNcoderConfiguration));
        PhoenixUtil.tryUntilOk(5, turret::optimizeBusUtilization);

        positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
        dutyCycleOut = new DutyCycleOut(0.0).withUpdateFreqHz(0.0);
        voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);

        position = turret.getPosition();
        velocityPerSec = turret.getVelocity();
        appliedVoltage = turret.getMotorVoltage();
        currentAmps = turret.getSupplyCurrent();
        torqueCurrent = turret.getTorqueCurrent();
        statorCurrent = turret.getStatorCurrent();
        tempCelcius = turret.getDeviceTemp();

        encoderPosition = cancoder.getPosition();
        encoderVelocity = cancoder.getVelocity();
        encoderVolts = cancoder.getSupplyVoltage();

        PhoenixUtil.registerSignals(bus, position, velocityPerSec, appliedVoltage, currentAmps, torqueCurrent, statorCurrent, tempCelcius);
        PhoenixUtil.registerSignals(bus, encoderPosition, encoderVelocity, encoderVolts);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        inputs.isConnected = BaseStatusSignal.isAllGood(position, velocityPerSec, appliedVoltage, currentAmps, torqueCurrent, statorCurrent, tempCelcius);
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.velocityRadPerSec = Rotation2d.fromRadians(velocityPerSec.getValueAsDouble());

        inputs.encoderConnected = BaseStatusSignal.isAllGood(encoderPosition, encoderVelocity, encoderVolts);
        inputs.encoderPosition = Rotation2d.fromRotations(encoderPosition.getValueAsDouble());
        inputs.encoderVelocity = Rotation2d.fromRadians(encoderVelocity.getValueAsDouble());
        inputs.encoderVoltage = encoderVolts.getValueAsDouble();
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        turret.setControl(
                isTorqueCurrent ? torqueCurrentFOC.withOutput(output) : voltageOut.withOutput(output)
        );
    }

    @Override
    public void runTurret(double output) {
        turret.setControl(
                dutyCycleOut.withOutput(output)
        );
    }


}
