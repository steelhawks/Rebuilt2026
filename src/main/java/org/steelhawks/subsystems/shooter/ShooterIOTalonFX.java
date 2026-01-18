package org.steelhawks.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX shooterMotor;

    private final StatusSignal<Angle> positionRad;
    private final StatusSignal<AngularVelocity> velocityRadPerSec;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;

    private final VoltageOut voltageOut;
    private final DutyCycleOut dutyCycleOut;

    public ShooterIOTalonFX() {

        shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID, ShooterConstants.CLAW_CANBUS);

        shooterMotor.setPosition(0.0);


        var motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
        motorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;

        shooterMotor.getConfigurator().apply(motorConfig);

        var slotConfigs = new Slot0Configs()
                .withKP(ShooterConstants.SHOOTER_KP.getAsDouble())
                .withKI(ShooterConstants.SHOOTER_KI.getAsDouble())
                .withKD(ShooterConstants.SHOOTER_KD.getAsDouble())
                .withKS(ShooterConstants.SHOOTER_KS.getAsDouble())
                .withKV(ShooterConstants.SHOOTER_KV.getAsDouble());



        shooterMotor.getConfigurator().apply(slotConfigs);

        voltageOut = new VoltageOut(0.0);
        dutyCycleOut = new DutyCycleOut(0.0);


        positionRad = shooterMotor.getPosition();
        velocityRadPerSec = shooterMotor.getVelocity();
        voltage = shooterMotor.getMotorVoltage();
        currentAmps = shooterMotor.getStatorCurrent();
        tempCelsius = shooterMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                positionRad,
                velocityRadPerSec,
                voltage,
                currentAmps,
                tempCelsius
        );

        shooterMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
                positionRad,
                velocityRadPerSec,
                voltage,
                currentAmps,
                tempCelsius
        ).isOK();
        inputs.positionRad = positionRad.getValueAsDouble();
        inputs.velocityRadPerSec = velocityRadPerSec.getValueAsDouble();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.currentsAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void runOpenLoop(double percentageOutput) {
        shooterMotor.setControl(voltageOut.withOutput(percentageOutput));
    }

    @Override
    public void runSpeed(double speed) {
        shooterMotor.setControl(dutyCycleOut.withOutput(speed));
    }

    @Override
    public void stop() {
        shooterMotor.stopMotor();
    }

}
