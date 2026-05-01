package org.steelhawks.subsystems.vision;

import org.steelhawks.RobotConfig;
import org.steelhawks.RobotState;
import org.steelhawks.subsystems.vision.VisionConstants.CameraConfig.CameraType;
import edu.wpi.first.wpilibj.RobotBase;
import org.steelhawks.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.VisionConstants.Factors.ObjFactors.LimelightFactors;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionIO;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionIOLimelight;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionIOPhoton;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionSim;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.HashSet;
import java.util.Set;

public class VisionConstants {

    public static LoggedTunableNumber baselineDropOdomFactor
        = new LoggedTunableNumber("Vision/BaselineDropOdomFactor", 0.8);

    public static final double NON_HUB_STDDEV_FACTOR = 1.1;

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.2; // sas 0.3
    public static final double MAX_ZERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double LINEAR_STD_DEV_BASELINE = 0.1; // Meters was 0.02
    public static final double ANGULAR_STD_DEV_BASELINE = 0.3; // Radians was 0.06

    // Multipliers to apply for MegaTag 2 observations
    public static final double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static final double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available

    // AprilTag layout
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final int[] BLUE_TAGS = new int[]  {
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };

    public static final int[] BLUE_HUB_ONLY = new int[] {
        18, 19, 20, 21, 24, 25, 26, 27
    };

    public static final int[] RED_TAGS = new int[] {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
    };

    public static final int[] RED_HUB_ONLY = new int[] {
        2, 3, 4, 5, 8, 9, 10, 11
    };

    public static final int[] ALL_ALLOWED_TAGS = new int[] {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };

    public static final Set<Integer> HUB_TAG_IDS = new HashSet<>();

    static {
        for (int id : BLUE_HUB_ONLY) HUB_TAG_IDS.add(id);
        for (int id : RED_HUB_ONLY) HUB_TAG_IDS.add(id);
    }

    public interface Factors {
        default Double[] getFactors() {
            return null;
        }

        class ObjFactors implements Factors {
            private final Double[] factors;

            public enum LimelightFactors {
                LIMELIGHT_4(82.0, 56.2),
                LIMELIGHT_3(62.5, 48.9),
                LIMELIGHT_2(62.5, 48.9)
                ;

                private final double horizontalFov;
                private final double verticalFov;

                LimelightFactors(double horizontalFov, double verticalFov) {
                    this.horizontalFov = horizontalFov;
                    this.verticalFov = verticalFov;
                }
            }

            /**
             * Only used for Limelight
             *
             * @param horizontalFov The horizontal FOV of the camera.
             * @param verticalFov The vertical FOV of the camera.
             * @param resolutionWidth The current resolution width selected in the Limelight config page.
             * @param resolutionHeight The current resolution height selected in the Limelight config page.
             * @param confidence The confidence level set in the Limelight config page.
             */
            public ObjFactors(double confidence, double horizontalFov, double verticalFov, double resolutionWidth, double resolutionHeight) {
                factors = new Double[] {
                    confidence,
                    horizontalFov,
                    verticalFov,
                    resolutionWidth,
                    resolutionHeight
                };
            }

            public ObjFactors(double confidence, LimelightFactors fov, double resolutionWidth, double resolutionHeight) {
                this(confidence, fov.horizontalFov, fov.verticalFov, resolutionWidth, resolutionHeight);
            }

            public ObjFactors() {
                this(0.0, 0.0, 0.0, 0.0, 0.0);
            }

            @Override
            public Double[] getFactors() {
                return factors;
            }
        }

        class StdDevFactors implements Factors {
            private final Double[] factors;

            public StdDevFactors(double stdDevLinear, double stdDevAngular) {
                factors = new Double[] {
                    stdDevLinear, stdDevAngular
                };
            }

            public StdDevFactors(double stdDevLinear) {
                this(stdDevLinear, stdDevLinear);
            }

            @Override
            public Double[] getFactors() {
                return factors;
            }
        }
    }

    /**
     * @param factors Standard deviation multipliers for each camera (Adjust to trust some cameras more than others) or for calculating object confidence
     */
    public record CameraConfig(
        String name, Transform3d robotToCamera, Factors factors, VisionConstants.CameraConfig.CameraType cameraType) {
        public enum CameraType {
            LIMELIGHT,
            PHOTON,
        }
    }

    private static final CameraConfig[] OMEGA_CAMERA_CONFIG = {
        new CameraConfig(
            "ov2311-west",
            new Transform3d(
                Units.inchesToMeters(-10.207), // 12.728 to the left
                Units.inchesToMeters(12.728), // 10.207 down
                Units.inchesToMeters(20.679),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-30.0),
                Units.degreesToRadians(90.0))),
            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
            CameraType.PHOTON
        ),
        new CameraConfig(
            "ov2311-northeast",
            new Transform3d(
                Units.inchesToMeters(-11.639), // 10.669 to the left
                Units.inchesToMeters(10.669), // 11.639 down
                Units.inchesToMeters(20.774),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-30.0),
                Units.degreesToRadians(-45.0))),
            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
            CameraType.PHOTON
        ),
//        new CameraConfig(
//            "ov9281-southwest",
//            new Transform3d(
//                Units.inchesToMeters(-13.303), // 12.333 to the left
//                Units.inchesToMeters(12.333), // 13.303 down
//                Units.inchesToMeters(20.774),
//            new Rotation3d(
//                Units.degreesToRadians(0.0),
//                Units.degreesToRadians(-30.0),
//                Units.degreesToRadians(-225.0))),
//            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
//            CameraType.PHOTON
//        ),
        new CameraConfig(
            "ov9281-east",
            new Transform3d(
                Units.inchesToMeters(-10.207), // 12.716 to the left
                Units.inchesToMeters(-12.716), // 10.207 down
                Units.inchesToMeters(20.679),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-30.0),
                Units.degreesToRadians(-90.0))),
            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
            CameraType.PHOTON
        ),
        new CameraConfig(
            "ov2311-northwest",
            new Transform3d(
                    Units.inchesToMeters(-11.669), // 10.676 to the right
                    Units.inchesToMeters(-10.676), // 11.669 down
                    Units.inchesToMeters(20.773),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-30.0),
                Units.degreesToRadians(45.0))),
            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
            CameraType.PHOTON
        ),
        new CameraConfig(
            "ov9281-southeast",
            new Transform3d(
                Units.inchesToMeters(-13.333), // 12.339 to the right
                Units.inchesToMeters(-12.339), // 13.333 down
                Units.inchesToMeters(20.774),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-30.0),
                Units.degreesToRadians(-135.0))),
            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
            CameraType.PHOTON
        ),
//        new CameraConfig(
//        "limelight-intake",
//            new Transform3d(
//                Units.inchesToMeters(-12.099), // 8.312 to the right
//                Units.inchesToMeters(-8.312), // 12.099 down
//                Units.inchesToMeters(18.785),
//                new Rotation3d(
//                    Units.degreesToRadians(0.0),
//                    Units.degreesToRadians(-30.0),
//                    Units.degreesToRadians(0.0))),
//            new Factors.StdDevFactors(2.0), // TODO: tune stddev factors
//            CameraType.LIMELIGHT
//        )
    };

    private static final CameraConfig[] LAST_YEAR_CAMERA_CONFIG = {
        new CameraConfig(
            "arducam-center-left",
            new Transform3d(
                // Left-Right: 0.098023
                // Front-Back: 3.174653
                // Up-Down: 6.950909
                // CHANGE DEPENDING HOW HIGH THE CAMERA IS MOUNTED ON THE BEAM
                // This assumes that the camera takes up holes 7 and 8 in the beam, counted from the top
                Units.inchesToMeters(3.174653),
                Units.inchesToMeters(-0.098023),
                Units.inchesToMeters(6.950909),
                new Rotation3d(
                    Units.degreesToRadians(0.058),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(-15))),
            new Factors.StdDevFactors(1.0),
            CameraType.PHOTON
        ),

        new CameraConfig(
            "ardu-center-right",
            // CENTER RIGHT MOUNT, the arducam that's placed on the right side of the beam inside the robot (30 DEGREE YAW) 20250323
            new Transform3d(
                // Left-Right: 9.062338
                // Front-Back: 3.134462
                // Up-Down: 6.900835
                // CHANGE DEPENDING HOW HIGH THE CAMERA IS MOUNTED ON THE BEAM
                // This assumes that the camera takes up holes 7 and 8 in the beam, counted from the top
                Units.inchesToMeters(3.134462),
                Units.inchesToMeters(-9.062338),
                Units.inchesToMeters(6.900835),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(30))),
            new Factors.StdDevFactors(1.0),
            CameraType.PHOTON
        ),

        new CameraConfig(
            "ardu-back-right",
            // BACK RIGHT (30 degree yaw, 20250323) (Same mount as previous Front Right from HVR, 20230307)
            new Transform3d(
                // Left-Right: 11.365372
                // Front-Back: 12.783453
                // Up-Down: 6.763611
                Units.inchesToMeters(-12.783453),
                Units.inchesToMeters(-11.365372),
                Units.inchesToMeters(6.763611),   // Z is from the top of the belly pan
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-120))),
            new Factors.StdDevFactors(8.0),
            CameraType.PHOTON
        ),

        new CameraConfig(
            "ardu-left",
            // ALGAE (Mounted on 1x1 beam running through the robot, such that the camera face is parallel to the reef when scoring algae)
            new Transform3d(
                // Left-Right: 10.251 left
                // Front-Back: 3.650 front
                // Up-Down: 6.940 up
                Units.inchesToMeters(3.650),
                Units.inchesToMeters(10.251),
                Units.inchesToMeters(6.940),   // Z is from the top of the belly pan
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(90))),
            new Factors.StdDevFactors(9.0),
            CameraType.PHOTON
        )
    };

    private static final CameraConfig[] OMEGA_OBJ_DETECT_CONFIG = {
        new CameraConfig(
            "limelight-intake",
            new Transform3d(
                Units.inchesToMeters(13.818), // 13.818 up
                0.0, // in the vertical middle
                Units.inchesToMeters(19.735),
                new Rotation3d(
                    0.0,
                    0.0,
                    Units.degreesToRadians(15.0))),
            new Factors.ObjFactors(0.8, LimelightFactors.LIMELIGHT_4, 1280.0, 800.0), // TODO: tune confidence factor
            CameraType.LIMELIGHT
        )
    };

    private static final CameraConfig[] CHASSIS_CAMERA_CONFIG = {
        new CameraConfig(
        "limelight-chassis",
        new Transform3d(),
        new Factors.StdDevFactors(3),
        CameraType.LIMELIGHT)
    };

    private static final CameraConfig[] CHASSIS_OBJ_DETECT_CONFIG = {};

    private static final CameraConfig[] ALPHA_CAMERA_CONFIG = {
        new CameraConfig(
            "FrontRight",
            new Transform3d(
                // 10.1 inches left from the center of robot
                // 10.4 inches upwards from the center of robot
                // 8.5 inches tall
                // measured by hand may be inaccurate
                Units.inchesToMeters(10.1), // prev -9.7
                Units.inchesToMeters(-10.4), // prev 11.125
                Units.inchesToMeters(8.5), // prev 9
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(0)
                )
            ),
            // TODO: Tune StdDevs
            new Factors.StdDevFactors(1),
            CameraType.PHOTON
        ),
        new CameraConfig(
            "FrontLeft",
            new Transform3d(
                Units.inchesToMeters(10.1),
                Units.inchesToMeters(10.4), // prev 10.65
                Units.inchesToMeters(8.5), // prev 9
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-15),
                    Units.degreesToRadians(0)
                )
            ),
            // TODO: Tune StdDevs
            new Factors.StdDevFactors(1),
            CameraType.PHOTON
        )
    };

    private static final CameraConfig[] ALPHA_OBJ_DETECT_CONFIG = {};

    private static final CameraConfig[] LAST_YEAR_OBJ_DETECT_CONFIG = {};

    public static CameraConfig[] getCameraConfig() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, SIMBOT -> OMEGA_CAMERA_CONFIG;
            case ALPHABOT -> ALPHA_CAMERA_CONFIG;
            case CHASSIS -> CHASSIS_CAMERA_CONFIG;
            case LAST_YEAR -> LAST_YEAR_CAMERA_CONFIG;
            case TEST_BOARD -> OMEGA_CAMERA_CONFIG;
        };
    }

    public static CameraConfig[] getObjDetectConfig() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, SIMBOT -> OMEGA_OBJ_DETECT_CONFIG;
            case ALPHABOT -> ALPHA_OBJ_DETECT_CONFIG;
            case CHASSIS -> CHASSIS_OBJ_DETECT_CONFIG;
            case LAST_YEAR -> LAST_YEAR_OBJ_DETECT_CONFIG;
            case TEST_BOARD -> null;
        };
    }

    public static VisionIO[] getIO() {
        CameraConfig[] config = getCameraConfig();
        VisionIO[] io = new VisionIO[config.length];
        for (int i = 0; i < config.length; i++) {
            if (RobotBase.isReal()) {
                switch (config[i].cameraType) {
                    case LIMELIGHT -> io[i] = new VisionIOLimelight(config[i].name, RobotState.getInstance()::getRotation);
                    case PHOTON -> io[i] = new VisionIOPhoton(config[i].name, config[i].robotToCamera);
                }
            } else if ((Constants.getRobot() == Constants.RobotType.SIMBOT && !RobotBase.isReal())
                || (RobotBase.isReal() && !RobotConfig.getConfig().hasVision)
            ) {
                io[i] = new VisionIOPhotonSim(
                    config[i].name,
                    config[i].robotToCamera,
                    Swerve.getDriveSimulation()::getSimulatedDriveTrainPose);
            } else if (Constants.getRobot() != Constants.RobotType.SIMBOT && !RobotBase.isReal()) {
                io[i] = new VisionIO() {};
            }
        }
        return io;
    }

    public static ObjectVisionIO[] getObjIO() {
        CameraConfig[] config = getObjDetectConfig();
        assert config != null;
        ObjectVisionIO[] io = new ObjectVisionIO[config.length];
        for (int i = 0; i < config.length; i++) {
            if (RobotBase.isReal()) {
                switch (config[i].cameraType) {
                    case LIMELIGHT -> io[i] = new ObjectVisionIOLimelight(config[i].name, i);
                    case PHOTON -> io[i] = new ObjectVisionIOPhoton(config[i].name, i);
                }
            } else if (Constants.getRobot() == Constants.RobotType.SIMBOT && !RobotBase.isReal()) {
                io[i] = new ObjectVisionSim(
                    config[i].name,
                    config[i].robotToCamera,
                    Swerve.getDriveSimulation()::getSimulatedDriveTrainPose);
            } else if (Constants.getRobot() != Constants.RobotType.SIMBOT && !RobotBase.isReal()) {
                io[i] = new ObjectVisionIO() {};
            }
        }
        return io;
    }
}
