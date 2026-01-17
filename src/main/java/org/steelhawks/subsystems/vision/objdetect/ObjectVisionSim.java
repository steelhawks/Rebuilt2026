package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.function.Supplier;

public class ObjectVisionSim implements ObjectVisionIO {

    private final String name;
    private final Transform3d robotToCamera;
    private final Supplier<Pose2d> poseSupplier;

    public ObjectVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.poseSupplier = poseSupplier;
    }
}
