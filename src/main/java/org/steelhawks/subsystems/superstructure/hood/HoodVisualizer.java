package org.steelhawks.subsystems.superstructure.hood;

import org.steelhawks.subsystems.superstructure.SuperstructureVisualizer;

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