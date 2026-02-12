package org.steelhawks.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;


public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX left_motor;
    private final TalonFX right_motor;

    private final TalonFX intake_motor;

    private final StatusSignal<Angle> rightExtensionPosition;
    private final StatusSignal<AngularVelocity> rightExtensionVelocityPerSec;
    private final StatusSignal<Current> rightExtensionCurrentAmps;
    private final StatusSignal<Voltage> rightExtensionAppliedVoltage;
    private final StatusSignal<Temperature> rightExtensionTemp;

    private final StatusSignal<Angle> leftExtensionPosition;
    private final StatusSignal<AngularVelocity> leftExtensionVelocityPerSec;
    private final StatusSignal<Current> leftExtensionCurrentAmps;
    private final StatusSignal<Voltage> leftExtensionAppliedVoltage;
    private final StatusSignal<Temperature> leftExtensionTemp;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocityPerSec;
    private final StatusSignal<Current> rollerCurrentAmps;
    private final StatusSignal<Voltage> rollerAppliedVoltage;
    private final StatusSignal<Temperature> rollerTemp;


}
