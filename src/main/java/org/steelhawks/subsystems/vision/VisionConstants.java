package org.steelhawks.subsystems.vision;

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


public class VisionConstants {

    // AprilTag layout (still used by FieldConstants / object detection).
    public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // AprilTag pose estimation (whitelisting, rejection, stddev weighting, camera
    // extrinsics) now lives entirely on the Orange Pi - see the :pi-service module
    // and org.steelhawks.common.VisionLinkConfig for the shared tag sets. Only the
    // object-detection config remains here.

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

    private static final CameraConfig[] CHASSIS_OBJ_DETECT_CONFIG = {};

    private static final CameraConfig[] ALPHA_OBJ_DETECT_CONFIG = {};

    private static final CameraConfig[] LAST_YEAR_OBJ_DETECT_CONFIG = {};

    public static CameraConfig[] getObjDetectConfig() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, SIMBOT -> OMEGA_OBJ_DETECT_CONFIG;
            case ALPHABOT -> ALPHA_OBJ_DETECT_CONFIG;
            case CHASSIS -> CHASSIS_OBJ_DETECT_CONFIG;
            case LAST_YEAR -> LAST_YEAR_OBJ_DETECT_CONFIG;
            case TEST_BOARD -> null;
        };
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
