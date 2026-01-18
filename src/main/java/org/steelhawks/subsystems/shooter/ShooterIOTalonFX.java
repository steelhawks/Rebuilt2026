package org.steelhawks.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX shooterMotor;

    private final StatusSignal<Angle> positionRad;
    private final StatusSignal<AngularVelocity> velocityRadPerSec;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;

    public ShooterIOTalonFX() {

        shooterMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID, ShooterConstants.CLAW_CANBUS);

        var motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));
        shooterMotor.getConfigurator().apply(motorConfig);

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

    }

}
