package org.steelhawks.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import static org.steelhawks.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

public class QuestNavImpl {

    private static final Matrix<N3, N1> STD_DEV = VecBuilder.fill(0.02, 0.02, 0.035);
    private static final double VISION_XY_DEVIATION_TOLERANCE = 0.3; // m
    private static final double VISION_THETA_DEVIATION_TOLERANCE = 0.087; // rad
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(0.0 , 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    private final Vision.VisionConsumer consumer;
    private final QuestNav nav;

    private boolean hasInitialPose = false;

    public QuestNavImpl(Vision.VisionConsumer consumer) {
        this.consumer = consumer;
        nav = new QuestNav();
    }

    public void periodic(List<Pose3d> visionPoses) {
        // required for library to work
        nav.commandPeriodic();

        Logger.recordOutput("QuestNav/Connected", nav.isConnected());
        Logger.recordOutput("QuestNav/Tracking", nav.isTracking());
        Logger.recordOutput("QuestNav/FrameCount", nav.getFrameCount().orElse(0));
        Logger.recordOutput("QuestNav/Battery", nav.getBatteryPercent().orElse(0));
        Logger.recordOutput("QuestNav/Latency", nav.getLatency());

        List<PoseFrame> allPoseFrames = new LinkedList<>();
        List<Pose2d> allQuestPoses = new LinkedList<>();
        List<Pose2d> allQuestPosesAccepted = new LinkedList<>();
        List<Pose2d> allQuestPosesRejected = new LinkedList<>();

        PoseFrame[] frames = nav.getAllUnreadPoseFrames();
        allPoseFrames.addAll(Arrays.asList(frames));
        allQuestPoses.addAll(
            Arrays.stream(frames)
                .map(frame -> frame.questPose3d().toPose2d())
                .toList());
        if (frames.length > 0) {
            for (int i = 0; i < frames.length; i++) {
                PoseFrame frame = frames[i];
                Pose2d robotPose =
                    frame.questPose3d()
                        .transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();
                // filtering / compare questnav position to vision positioning
                final boolean rejectPose =
                    Constants.loggedValue("HasInitialPose", !hasInitialPose)
                    || Constants.loggedValue("InXFieldMin",robotPose.getX() < 0.0)
                    || Constants.loggedValue("InXFieldMax", robotPose.getX() > APRIL_TAG_LAYOUT.getFieldLength())
                    || Constants.loggedValue("InYFieldMin", robotPose.getY() < 0.0)
                    || Constants.loggedValue("InYFieldMax", robotPose.getY() > APRIL_TAG_LAYOUT.getFieldWidth())
                    || Constants.loggedValue("OutOfVisionTolerance", outOfVisionTolerance(frame.questPose3d().toPose2d(),
                        visionPoses.stream().findAny().orElse(new Pose3d()).toPose2d()))
                    || Constants.loggedValue("QuestTracking", !nav.isTracking())
                    || Constants.loggedValue("QuestConnected", !nav.isConnected());
                if (outOfVisionTolerance(frame.questPose3d().toPose2d(),
                    visionPoses.stream().findAny().orElse(new Pose3d()).toPose2d())
                ) {
                    boolean visionEstimateOk =
                        Constants.loggedValue("EstimateOk/InXMax", visionPoses.stream().findAny().orElse(new Pose3d()).getX() < 0.0)
                        || Constants.loggedValue("EstimateOk/InXMax", visionPoses.stream().findAny().orElse(new Pose3d()).getX() > APRIL_TAG_LAYOUT.getFieldLength())
                        || Constants.loggedValue("EstimateOk/InYMin", visionPoses.stream().findAny().orElse(new Pose3d()).getY() < 0.0)
                        || Constants.loggedValue("EstimateOk/InYMax", visionPoses.stream().findAny().orElse(new Pose3d()).getY() > APRIL_TAG_LAYOUT.getFieldWidth());
                    if (visionEstimateOk && !visionPoses.isEmpty()) {
                        setPose(visionPoses.get(0).toPose2d());
                    }
                }
                Logger.recordOutput("QuestNav/UnfilteredPose", robotPose);
                if (rejectPose) {
                    allQuestPosesRejected.add(robotPose);
                } else {
                    allQuestPosesAccepted.add(robotPose);
                }
                if (rejectPose) {
                    continue;
                }
                consumer.accept(robotPose, frame.dataTimestamp(), STD_DEV);

                Logger.recordOutput("QuestNav/AllPoses", allQuestPoses.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/RejectedPoses", allQuestPosesRejected.toArray(new Pose2d[0]));
                Logger.recordOutput("QuestNav/AcceptedPoses", allQuestPosesAccepted.toArray(new Pose2d[0]));
            }
        }
    }

    private boolean outOfVisionTolerance(Pose2d questPose, Pose2d visionPose) {
        return Constants.loggedValue("VisionCheck/InXTolWithVision", !(Math.abs(questPose.getX() - visionPose.getX()) <= VISION_XY_DEVIATION_TOLERANCE))
            || Constants.loggedValue("VisionCheck/InYTolWithVision", !(Math.abs(questPose.getY() - visionPose.getY()) <= VISION_XY_DEVIATION_TOLERANCE))
            || Constants.loggedValue("VisionCheck/InThetaTolWithVision", !(Math.abs(questPose.getRotation().getRadians() - visionPose.getRotation().getRadians())
                <= VISION_THETA_DEVIATION_TOLERANCE));
    }

    public boolean isRunning() {
        return nav.isTracking() && nav.isConnected();
    }

    public void setPose(Pose2d pose) {
        hasInitialPose = true;
        nav.setPose(new Pose3d(pose).transformBy(ROBOT_TO_QUEST));
    }
}
