package org.steelhawks.subsystems.vision;

import static org.steelhawks.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.*;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotState.PoseObservationType;
import org.steelhawks.util.LoopTimeUtil;
import org.steelhawks.util.VirtualSubsystem;

public class Vision extends SubsystemBase {
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private static int[] allowedTagIds;
    private final boolean useQuestNav;

    private final QuestNavImpl questNav;

    public Vision() {
        this(false);
    }

    public Vision(boolean useQuestNav) {
        this.useQuestNav = useQuestNav;
        this.io = VisionConstants.getIO();

        if (useQuestNav) {
            questNav = new QuestNavImpl();
        } else {
            questNav = null;
        }

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                new Alert(
                    "Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }
        whitelistTagIds(ALL_ALLOWED_TAGS);
    }

    public static void whitelistTagIds(int... tagIds) {
        allowedTagIds = tagIds;
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    /**
     * Returns the fiducial ID of the best target.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public int getTargetId(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.fiducialId();
    }

    /**
     * Returns the Y angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetY(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.ty();
    }

    /**
     * Returns true if an AprilTag target is in view.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public boolean hasTarget(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.fiducialId() != -1;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            if (Toggles.Vision.camerasEnabled.get(io[i].getName()).get()) {
                io[i].updateInputs(inputs[i]);
            }
            Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
        }
        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            if (!Toggles.Vision.camerasEnabled.get(io[cameraIndex].getName()).get()) {
                continue;
            }

            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                // Check if tag ID is whitelisted
                boolean isWhitelisted = false;
                for (int allowedTagId : allowedTagIds) {
                    if (tagId == allowedTagId) {
                        isWhitelisted = true;
                        break;
                    }
                }
                if (!isWhitelisted) {
                    return;
                }
                var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                    observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > MAX_ZERROR // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor =
                    Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
                    angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                }
                if (cameraIndex < Objects.requireNonNull(VisionConstants.getCameraConfig()).length) {
                    linearStdDev *= getCameraConfig()[cameraIndex].factors().getFactors()[0]
                        * (RobotContainer.s_Swerve.isOnBump() ? VisionConstants.baselineDropOdomFactor.get() : 1.0);
                    angularStdDev *= getCameraConfig()[cameraIndex].factors().getFactors()[1]
                        * (RobotContainer.s_Swerve.isOnBump() ? VisionConstants.baselineDropOdomFactor.get() : 1.0);
                }
                if (useQuestNav && !Robot.isFirstRun()) {
                    assert questNav != null;
                    if (questNav.isRunning()) {
                        linearStdDev = 2601_2601;
                        angularStdDev = 2601_2601;
                    }
                }

                // Send vision observation
                RobotState.getInstance().addVisionObservation(
                    new RobotState.VisionObservation(
                        observation.timestamp(),
                        observation.pose().toPose2d(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                    )
                );
            }
            LoopTimeUtil.record("Normal Vision");


            // Log camera datadata
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/RobotPoses",
                robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
            "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        LoopTimeUtil.record("Vision");

        if (questNav != null) {
            // make it so that in debug mode you can automatically reset pose from vision when disabled/ or make it a toggle
            if (DriverStation.isDisabled() && Robot.isFirstRun() && Constants.loggedValue("RobotPosesEmpty", !allRobotPoses.isEmpty())) {
                questNav.setPose(allRobotPosesAccepted.get(0).toPose2d());
            }
            questNav.periodic(allRobotPoses);
            LoopTimeUtil.record("QuestNav");
        }
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
