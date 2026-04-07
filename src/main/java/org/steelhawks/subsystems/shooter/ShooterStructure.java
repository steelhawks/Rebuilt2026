package org.steelhawks.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.littletonrobotics.junction.Logger;

public class ShooterStructure {
    private static final InterpolatingDoubleTreeMap VELOCITY_MAP
        = new InterpolatingDoubleTreeMap();

    static {
        VELOCITY_MAP.put(60.5, 15.0);
        VELOCITY_MAP.put(73.1, 20.0);
    }

    public double getDesiredVelocityLUT(double distanceInches) {
        return VELOCITY_MAP.get(distanceInches);
    }

}
