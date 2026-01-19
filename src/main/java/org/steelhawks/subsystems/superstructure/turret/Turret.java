package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final TurretIO io;

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
