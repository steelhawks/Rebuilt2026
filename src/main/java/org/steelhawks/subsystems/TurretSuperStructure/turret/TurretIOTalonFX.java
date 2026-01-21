package org.steelhawks.subsystems.TurretSuperStructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public class TurretIOTalonFX implements TurretIO {

    private final TalonFX turretMotor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;

    private final DutyCycleOut dutyCycleOut;

    public TurretIOTalonFX() {
        turretMotor = new TalonFX(TurretConstants.MOTOR_ID, TurretConstants.TURRET_CANBUS);
        dutyCycleOut = new DutyCycleOut(0.0);

        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

        motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(TurretConstants.GEAR_RATIO);

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        turretMotor.getConfigurator().apply(motorConfig);

        var slotConfigs = new Slot0Configs()
                .withKP(TurretConstants.SHOOTER_KP.getAsDouble())
                .withKI(TurretConstants.SHOOTER_KI.getAsDouble())
                .withKD(TurretConstants.SHOOTER_KD.getAsDouble())
                .withKS(TurretConstants.SHOOTER_KS.getAsDouble())
                .withKV(TurretConstants.SHOOTER_KV.getAsDouble())
                .withKA(TurretConstants.SHOOTER_KA.getAsDouble());

        turretMotor.getConfigurator().apply(slotConfigs);

        position = turretMotor.getPosition();
        appliedVolts = turretMotor.getMotorVoltage();
        currentAmps = turretMotor.getStatorCurrent();
        tempCelsius = turretMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                appliedVolts,
                currentAmps,
                tempCelsius
        );

    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
                position,
                appliedVolts,
                currentAmps,
                tempCelsius
        ).isOK();

        inputs.amountTurned = position.getValueAsDouble() * 360.0;
        inputs.appliedVoltage = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

    }

    @Override
    public void turnTurret(TurretIOInputs inputs, double percentageOutput) {
        if (inputs.amountTurned <= 360.0) {
            turretMotor.setControl(dutyCycleOut.withOutput(percentageOutput));
        } else {
            stop();
        }
    }

    @Override
    public void stop() {
        turretMotor.stopMotor();
    }
}
