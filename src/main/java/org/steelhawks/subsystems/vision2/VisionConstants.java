package org.steelhawks.subsystems.vision2;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static final String LIMELIGHT_NAME = "LIMELIGHT";
    public static final String PHOTON_NAME = "PHOTON";

    public static final Transform3d ROBOT_TO_FRONT = new Transform3d(
            new Translation3d(0.3, 0.0, 0.2),
            new Rotation3d(0.0, Units.degreesToRadians(-15.0), 0.0)
    );

    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_POSE_TRUST_DISTANCE_METERS = 4.0;

    // Standard deviations for pose estimation (x, y, theta)
    public static final double[] SINGLE_TAG_STD_DEVS = {4.0, 4.0, 8.0};
    public static final double[] MULTI_TAG_STD_DEVS  = {0.5, 0.5, 1.0};

    // Pipeline indices (Limelight)
    public static final int APRILTAG_PIPELINE = 0;
    public static final int RETROREFLECTIVE_PIPELINE = 1;



}
