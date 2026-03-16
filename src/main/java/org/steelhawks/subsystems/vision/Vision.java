package org.steelhawks.subsystems.vision;

import static org.steelhawks.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.*;

import java.util.*;

import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotState.PoseObservationType;
import org.steelhawks.util.LoopTimeUtil;

public class Vision extends SubsystemBase {
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private static final Set<Integer> allowedTagIds = new HashSet<>();
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

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                new Alert(
                    "Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }
        whitelistTagIds(ALL_ALLOWED_TAGS);
    }

    public static void whitelistTagIds(int... tagIds) {
        allowedTagIds.clear();
        for (int id : tagIds) {
            allowedTagIds.add(id);
        }
    }

    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public int getTargetId(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.fiducialId();
    }

    public Rotation2d getTargetY(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.ty();
    }

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
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            if (!Toggles.Vision.camerasEnabled.get(io[cameraIndex].getName()).get()) {
                continue;
            }
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : inputs[cameraIndex].tagIds) {
                if (!allowedTagIds.contains(tagId)) continue;
                var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
                tagPose.ifPresent(tagPoses::add);
            }

            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean rejectPose =
                    observation.tagCount() == 0
                        || (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY)
                        || Math.abs(observation.pose().getZ()) > MAX_ZERROR
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();

                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                if (rejectPose) continue;

                double stdDevFactor =
                    Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
                    angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                }
                if (cameraIndex < Objects.requireNonNull(VisionConstants.getCameraConfig()).length) {
                    double cameraLinearFactor = getCameraConfig()[cameraIndex].factors().getFactors()[0];
                    double cameraAngularFactor = getCameraConfig()[cameraIndex].factors().getFactors()[1];
                    if (RobotContainer.s_Swerve.isOnBump()) { // trust vision as much as possible
                        linearStdDev *= VisionConstants.baselineDropOdomFactor.get();
                        angularStdDev *= VisionConstants.baselineDropOdomFactor.get();
                    } else {
                        linearStdDev *= cameraLinearFactor;
                        angularStdDev *= cameraAngularFactor;
                    }
                }
                if (useQuestNav && !Robot.isFirstRun()) {
                    assert questNav != null;
                    if (questNav.isRunning()) {
                        linearStdDev = 2601_2601;
                        angularStdDev = 2601_2601;
                    }
                }
                RobotState.getInstance().addVisionObservation(
                    new RobotState.VisionObservation(
                        observation.timestamp(),
                        observation.pose().toPose2d(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                    )
                );
            }
            LoopTimeUtil.record("Normal Vision");

            if (Toggles.debugMode.get() || RobotBase.isSimulation()) {
                Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
                Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
                Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
                Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
            }
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary poses
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

        // Log camera poses and rays from cameras to tags
        if (Toggles.debugMode.get() || RobotBase.isSimulation()) {
            CameraConfig[] cameraConfigs = VisionConstants.getCameraConfig();
            if (cameraConfigs != null) {
                Pose2d robotPose2d = RobotState.getInstance().getEstimatedPose();
                Pose3d robotPose3d = new Pose3d(
                    robotPose2d.getX(),
                    robotPose2d.getY(),
                    0.0,
                    new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));
                // camera positions on the robot
                Pose3d[] cameraPoses = new Pose3d[cameraConfigs.length];
                for (int i = 0; i < cameraConfigs.length; i++) {
                    cameraPoses[i] = robotPose3d.transformBy(cameraConfigs[i].robotToCamera());
                }
                Logger.recordOutput("Vision/Summary/CameraPoses", cameraPoses);
                // log a separate trajectory per camera, only to tags that camera sees
                for (int i = 0; i < cameraConfigs.length; i++) {
                    Pose3d cameraPose = cameraPoses[i];
                    List<Pose3d> cameraRays = new LinkedList<>();
                    // get tag poses seen by this camera
                    for (int tagId : inputs[i].tagIds) {
                        var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
                        if (tagPose.isPresent()) {
                            cameraRays.add(cameraPose);
                            cameraRays.add(tagPose.get());
                        }
                    }
                    Logger.recordOutput("Vision/Camera" + i + "/CameraRays", cameraRays.toArray(new Pose3d[0]));
                }
            }
        }
        LoopTimeUtil.record("Vision");
        if (questNav != null) {
            if (DriverStation.isDisabled() && Robot.isFirstRun() && Constants.loggedValue("RobotPosesEmpty", !allRobotPosesAccepted.isEmpty())) {
                questNav.setPose(allRobotPosesAccepted.get(0).toPose2d());
            }
            questNav.periodic(allRobotPoses);
            LoopTimeUtil.record("QuestNav");
        }
    }
}
