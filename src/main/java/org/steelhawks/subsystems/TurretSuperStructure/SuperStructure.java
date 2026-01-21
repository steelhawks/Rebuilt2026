package org.steelhawks.subsystems.TurretSuperStructure;


import org.steelhawks.subsystems.TurretSuperStructure.shooter.ShooterIO;
import org.steelhawks.subsystems.TurretSuperStructure.turret.TurretIO;

public class SuperStructure {

    private final ShooterIO shooterio;
    private final TurretIO turretio;

    public SuperStructure(ShooterIO shooterio, TurretIO turretio) {
        this.shooterio = shooterio;
        this.turretio = turretio;
    }

}
