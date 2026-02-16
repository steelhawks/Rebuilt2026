package org.steelhawks.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Timer;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeState intakeState;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private Timer timer = new Timer();

    enum IntakeState {
        IDLE,
        INTAKING,
        EJECTING,
        HOLDING,
        HOMING,
    }

    public Intake(IntakeIO io) {
        this.io = io;

        io.setExtensionBrakeMode(true);
        io.setRollerBrakeMode(false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/State", intakeState.toString());
        Logger.processInputs("Intake", inputs);


    }
}
