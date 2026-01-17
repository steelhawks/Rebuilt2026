package org.steelhawks.subsystems.vision;

import org.dyn4j.geometry.Rotation;
import org.steelhawks.subsystems.vision.VisionConstants.CameraConfig.CameraType;
import edu.wpi.first.wpilibj.RobotBase;
import org.steelhawks.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.VisionConstants.Factors.ObjFactors.LimelightFactors;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionIO;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionIOLimelight;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionIOPhoton;
import org.steelhawks.subsystems.vision.objdetect.ObjectVisionSim;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static int[] ALL_ALLOWED_TAGS = new int[] {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };

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
            "arducam-center-mount",
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
            "arducam-center-right",
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
            "arducam-back-right",
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
            "arducam-algae",
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
            "limelight-coral",
//            Constants.fromOnshapeCoordinates(4.469, 9.261, 21.578, 15.0, -20.0, 0.0),
//            Constants.fromOnshapeCoordinates(5.531, -15.261, -23.578 - 1.414, 15.0, -20.0, 0.0),
            new Transform3d(
                Units.inchesToMeters(15.261),
                Units.inchesToMeters(5.531),
                Units.inchesToMeters(23.578 + 1.414),  // height (keep as is)
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-20.0),
                    Units.degreesToRadians(15.0)
                )
            ),
            new Factors.ObjFactors(0.8, LimelightFactors.LIMELIGHT_4, 1280.0, 800.0),
            CameraType.LIMELIGHT
        )
    };

    private static final CameraConfig[] ALPHA_CAMERA_CONFIG = {
        new CameraConfig(
            "limelight-coral",
            new Transform3d(),
            new Factors.StdDevFactors(3.0),
            CameraType.LIMELIGHT
        )
    };

    private static final CameraConfig[] LAST_YEAR_CAMERA_CONFIG = {
        new CameraConfig(
            "limelight-shooter",
            new Transform3d(),
            new Factors.StdDevFactors(1.2),
            CameraType.LIMELIGHT
        ),
        new CameraConfig(
            "limelight",
            new Transform3d(),
            new Factors.StdDevFactors(1.3),
            CameraType.LIMELIGHT
        )
    };

    public static CameraConfig[] getCameraConfig() {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> ALPHA_CAMERA_CONFIG;
            case LAST_YEAR -> LAST_YEAR_CAMERA_CONFIG;
            default -> OMEGA_CAMERA_CONFIG;
        };
    }

    public static CameraConfig[] getObjDetectConfig() {
        return switch (Constants.getRobot()) {
            case SIMBOT, OMEGABOT -> OMEGA_OBJ_DETECT_CONFIG;
            default -> null;
        };
    }

    public static VisionIO[] getIO() {
        CameraConfig[] config = getCameraConfig();
        VisionIO[] io = new VisionIO[config.length];
        for (int i = 0; i < config.length; i++) {
            if (RobotBase.isReal()) {
                switch (config[i].cameraType) {
                    case LIMELIGHT -> io[i] = new VisionIOLimelight(config[i].name, RobotContainer.s_Swerve::getRotation);
                    case PHOTON -> io[i] = new VisionIOPhoton(config[i].name, config[i].robotToCamera);
                }
            } else if (Constants.getRobot() == Constants.RobotType.SIMBOT && !RobotBase.isReal()) {
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

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.3;
    public static double MAX_ZERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STD_DEV_MEGATAG2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available

}
