package org.steelhawks.subsystems.shooterSuperstructure.hood;

import org.steelhawks.subsystems.shooterSuperstructure.SuperstructureVisualizer;

import java.util.function.Supplier;

public class HoodVisualizer {
    public HoodVisualizer(Supplier<Double> hoodRad) {
        SuperstructureVisualizer.getInstance().setHoodSupplier(hoodRad);
    }
    public void update() {
        // intentionally empty - TurretVisualizer drives the update call
        // so we don't double-log per loop
    }
}