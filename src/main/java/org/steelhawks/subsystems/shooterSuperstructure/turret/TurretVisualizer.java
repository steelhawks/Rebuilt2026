package org.steelhawks.subsystems.shooterSuperstructure.turret;

import edu.wpi.first.math.geometry.Pose2d;
import org.steelhawks.subsystems.shooterSuperstructure.SuperstructureVisualizer;

import java.util.function.Supplier;

public class TurretVisualizer {

    public TurretVisualizer(Supplier<Double> turretRad, Supplier<Pose2d> robotPose) {
        SuperstructureVisualizer.getInstance().setTurretSupplier(turretRad);
        SuperstructureVisualizer.getInstance().setRobotPoseSupplier(robotPose);
    }

    public void update() {
        SuperstructureVisualizer.getInstance().update();
    }
}