package org.steelhawks.subsystems.superstructure.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import org.steelhawks.RobotConfig;

public class TurretIOTalonFX {
    private final TalonFX turret_motor;

    private final TalonFXConfiguration config;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocityRadPerSec;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> torqueCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> tempCelsius;

    private final PositionTorqueCurrentFOC left_motor_torque_current;
    private final PositionTorqueCurrentFOC right_motor_torque_current;
    private final TorqueCurrentFOC left_extension_torque_current;
    private final TorqueCurrentFOC right_extension_torque_current;

    private final VoltageOut extensionVoltage;
    private final DutyCycleOut extensionPercentOut;

    private final CANcoderConfiguration cancoderConfiguration;
    private final CANcoder CANcoder;

    public TurretIOTalonFX(CANBus bus, )


}
