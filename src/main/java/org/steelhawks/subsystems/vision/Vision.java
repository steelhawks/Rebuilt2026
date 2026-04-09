package org.steelhawks.subsystems.vision;

import static org.steelhawks.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.*;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotState.PoseObservationType;
import org.steelhawks.util.LoopTimeUtil;

public class Vision extends SubsystemBase {
    // 2 threads matches RoboRIO 2s dual-core CPU lets updateInputs() calls for
    // all cameras run in parallel while the main thread is blocked on Future.get().
    private static final ExecutorService visionExecutor =
        Executors.newFixedThreadPool(2);

    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    // Double-buffer per camera: worker writes ONLY to backBuffers[i].
    // Main thread swaps after Future.get() guarantees the worker is done.
    // This eliminates the data race that was corrupting inputs[] mid-read.
    private final VisionIOInputsAutoLogged[] backBuffers;

    private static final Set<Integer> allowedTagIds = new HashSet<>();
    private final boolean useQuestNav;

    private final QuestNavImpl questNav;
    private final Debouncer stableTagDebouncer =
        new Debouncer(0.1, DebounceType.kFalling);
    private boolean tagStable = false;

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
        this.backBuffers = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            backBuffers[i] = new VisionIOInputsAutoLogged();
        }

        whitelistTagIds(ALL_ALLOWED_TAGS);
    }

    public static void whitelistTagIds(int... tagIds) {
        allowedTagIds.clear();
        for (int id : tagIds) {
            allowedTagIds.add(id);
        }
    }

    /**
     * Returns true if at least one of the tags currently seen by this camera is whitelisted.
     * Used to gate pose observations — if a camera only sees opposing alliance tags,
     * all of its observations are rejected to prevent pose corruption.
     */
    private boolean cameraHasAllowedTag(int cameraIndex) {
        for (int tagId : inputs[cameraIndex].tagIds) {
            if (allowedTagIds.contains(tagId)) {
                return true;
            }
        }
        return false;
    }

    public boolean hasStableTag() {
        return tagStable;
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
        // Read toggle state on the main thread before handing off to workers.
        // NT value reads are not thread-safe; do this once and pass the result in.
        boolean[] cameraEnabled = new boolean[io.length];
        for (int i = 0; i < io.length; i++) {
            cameraEnabled[i] = Toggles.Vision.camerasEnabled.get(io[i].getName()).get();
        }

        // Submit updateInputs() for each enabled camera to the thread pool.
        // Workers write ONLY to backBuffers[idx] inputs[] is never touched
        // off the main thread, eliminating the previous data race.
        @SuppressWarnings("unchecked")
        Future<Void>[] futures = new Future[io.length];
        for (int i = 0; i < io.length; i++) {
            if (cameraEnabled[i]) {
                final int idx = i;
                futures[idx] = visionExecutor.submit(() -> {
                    io[idx].updateInputs(backBuffers[idx]);
                    return null;
                });
            }
        }

        // Wait for each camera, then swap on the main thread.
        // Future.get() provides a happens before guarantee, so the swap is safe
        // and inputs[] is always consistent when processInputs() is called below.
        for (int i = 0; i < io.length; i++) {
            if (futures[i] != null) {
                try {
                    futures[i].get(15, TimeUnit.MILLISECONDS);
                    // Swap only after the worker is confirmed done
                    VisionIOInputsAutoLogged tmp = inputs[i];
                    inputs[i] = backBuffers[i];
                    backBuffers[i] = tmp;
                } catch (Exception e) {
                    // Timed out or threw keep stale inputs from last cycle
                    System.out.println(
                        "Vision camera " + i + " update timed out: " + e.getMessage());
                }
            }
            Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
        }

        List<Pose3d> allTagPoses = new ArrayList<>();
        List<Pose3d> allRobotPoses = new ArrayList<>();
        List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
        List<Pose3d> allRobotPosesRejected = new ArrayList<>();
        boolean hasAllowedTag = false;

        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Use the snapshot taken at the top of periodic() do not rerad NT here
            if (!cameraEnabled[cameraIndex]) {
                continue;
            }

            List<Pose3d> tagPoses = new ArrayList<>();
            List<Pose3d> robotPoses = new ArrayList<>();
            List<Pose3d> robotPosesAccepted = new ArrayList<>();
            List<Pose3d> robotPosesRejected = new ArrayList<>();

            // Only log tags that are whitelisted
            for (int tagId : inputs[cameraIndex].tagIds) {
                if (!allowedTagIds.contains(tagId)) continue;
                var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
                tagPose.ifPresent(tagPoses::add);
            }

            // Gate the entire camera's pose observations if it sees no whitelisted tags.
            if (!cameraHasAllowedTag(cameraIndex)) {
                if (Toggles.debugMode.get() || RobotBase.isSimulation()) {
                    Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
                    Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", new Pose3d[0]);
                    Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", new Pose3d[0]);
                    Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", new Pose3d[0]);
                }
                continue;
            }
            hasAllowedTag = true;

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
                if (observation.tagCount() == 1) stdDevFactor *= 3.0;

                boolean hasHubTag = false;
                for (int tagId : inputs[cameraIndex].tagIds) {
                    if (VisionConstants.HUB_TAG_IDS.contains(tagId)) {
                        hasHubTag = true;
                        break;
                    }
                }
                if (!hasHubTag) stdDevFactor *= NON_HUB_STDDEV_FACTOR;

                double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
                    angularStdDev *= ANGULAR_STD_DEV_MEGATAG2_FACTOR;
                }

                if (cameraIndex < Objects.requireNonNull(VisionConstants.getCameraConfig()).length) {
                    double cameraLinearFactor = getCameraConfig()[cameraIndex].factors().getFactors()[0];
                    double cameraAngularFactor = getCameraConfig()[cameraIndex].factors().getFactors()[1];
                    if (RobotContainer.s_Swerve.isOnBump()) {
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

        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));

        // Log summary poses
        if (Toggles.debugMode.get() || RobotBase.isSimulation()) {
            Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
        }

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

                Pose3d[] cameraPoses = new Pose3d[cameraConfigs.length];
                for (int i = 0; i < cameraConfigs.length; i++) {
                    cameraPoses[i] = robotPose3d.transformBy(cameraConfigs[i].robotToCamera());
                }
                Logger.recordOutput("Vision/Summary/CameraPoses", cameraPoses);

                for (int i = 0; i < cameraConfigs.length; i++) {
                    Pose3d cameraPose = cameraPoses[i];
                    List<Pose3d> cameraRays = new ArrayList<>();
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

        tagStable = stableTagDebouncer.calculate(hasAllowedTag);
        LoopTimeUtil.record("Vision");

        if (questNav != null) {
            if (DriverStation.isDisabled()
                && Robot.isFirstRun()
                && Constants.loggedValue("RobotPosesEmpty", !allRobotPosesAccepted.isEmpty())) {
                questNav.setPose(allRobotPosesAccepted.get(0).toPose2d());
            }
            questNav.periodic(allRobotPoses);
            LoopTimeUtil.record("QuestNav");
        }
    }
}
