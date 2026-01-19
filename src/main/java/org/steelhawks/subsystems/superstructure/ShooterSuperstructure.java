package org.steelhawks.subsystems.superstructure;

import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.pivot.Pivot;
import org.steelhawks.subsystems.superstructure.turret.Turret;

public class ShooterSuperstructure {

    private final Flywheel flywheel;
    private final Turret turret;
    private final Pivot pivot;

    public ShooterSuperstructure(
        Flywheel flywheel, Turret turret, Pivot pivot
    ) {
        this.flywheel = flywheel;
        this.turret = turret;
        this.pivot = pivot;
    }

}
