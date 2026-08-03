package org.steelhawks.pi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.steelhawks.common.VisionLinkConfig;
import org.steelhawks.pi.gtsam.NativePoseEstimator;
import org.steelhawks.pi.link.OdomSample;
import org.steelhawks.pi.link.RioLink;
import org.steelhawks.pi.vision.AcceptedObservation;
import org.steelhawks.pi.vision.CameraObservation;
import org.steelhawks.pi.vision.PhotonVisionIO;
import org.steelhawks.pi.vision.PhotonVisionIOReal;
import org.steelhawks.pi.vision.VisionFilter;
import org.steelhawks.proto.AllianceColor;

/**
 * The Pi vision/pose service loop. Each cycle: drain odometry from the RIO and
 * feed it as BetweenFactors, poll + filter PhotonVision and feed accepted tags
 * as PriorFactors, solve the GTSAM graph, and send the fused pose back. Output
 * is withheld until the graph is anchored.
 */
public class PiRobot extends LoggedRobot {

    private RioLink link;
    private PhotonVisionIO photon;
    private NativePoseEstimator estimator;

    // Previous cumulative odometry pose, differenced into between-factors.
    private Pose2d lastOdomPose = null;
    private long appliedResetSeqnum = -1;

    // Context carried between odom and vision within a cycle
    private AllianceColor alliance = AllianceColor.ALLIANCE_UNKNOWN;
    private boolean isOnBump = false;
    private double lastOdomTimestamp = 0.0;
    private Pose2d currentEstimate = new Pose2d();
    private long txSeqnum = 0;

    // Per-camera accept/reject tallies for the observation log.
    private final int[] acceptedCount = new int[PiVisionConstants.CAMERAS.length];
    private final int[] rejectedCount = new int[PiVisionConstants.CAMERAS.length];

    @Override
    public void robotInit() {
        Logger.recordMetadata("Service", "poselink-pi");
        Logger.recordMetadata("ConfigHash", Long.toString(VisionLinkConfig.CONFIG_HASH));
        Logger.addDataReceiver(new WPILOGWriter());
        if (PiVisionConstants.ALLOW_PUBLISH_NT4_TELEMETRY) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start();

        link = new RioLink();
        photon = new PhotonVisionIOReal();
        estimator = new NativePoseEstimator(PiVisionConstants.LAG_SECONDS);
    }

    @Override
    public void robotPeriodic() {
        for (OdomSample s : link.drain()) {
            ingestOdometry(s);
        }
        for (CameraObservation obs : photon.poll()) {
            VisionFilter.Result res = VisionFilter.filter(obs, alliance, isOnBump, currentEstimate);
            logObservation(obs, res);
            if (res.isAccepted()) {
                AcceptedObservation a = res.accepted();
                estimator.addVisionMeasurement(
                    a.timestamp(), a.x(), a.y(), a.theta(), a.varX(), a.varY(), a.varTheta());
            }
        }

        long t0 = System.nanoTime();
        int status = estimator.update();
        double solveMs = (System.nanoTime() - t0) / 1e6;

        NativePoseEstimator.Result r = estimator.getResult();
        if (status == NativePoseEstimator.STATUS_OK) {
            currentEstimate = new Pose2d(r.x(), r.y(), new Rotation2d(r.theta()));
            link.sendFusedPose(
                txSeqnum++,
                lastOdomTimestamp,
                currentEstimate,
                quality(r),
                r.covXX(), r.covYY(), r.covTheta(),
                VisionLinkConfig.CONFIG_HASH,
                solveMs,
                appliedResetSeqnum);
        }

        Logger.recordOutput("PoseLinkPi/Status", status);
        Logger.recordOutput("PoseLinkPi/FusedPose", currentEstimate);
        Logger.recordOutput("PoseLinkPi/SolveMs", solveMs);
        Logger.recordOutput("PoseLinkPi/Nodes", r.nodeCount());
        Logger.recordOutput("PoseLinkPi/Factors", r.factorCount());
        Logger.recordOutput("PoseLinkPi/DroppedOdom", link.droppedCount());
        Logger.recordOutput("PoseLinkPi/CamerasConnected", photon.allConnected());
    }

    private void logObservation(CameraObservation obs, VisionFilter.Result res) {
        int i = obs.cameraIndex();
        String base = "PoseLinkPi/Cam/" + PiVisionConstants.CAMERAS[i].name() + "/";

        if (res.isAccepted()) {
            acceptedCount[i]++;
        } else {
            rejectedCount[i]++;
        }

        Pose2d camPose = obs.robotPose().toPose2d();
        Logger.recordOutput(base + "Pose", camPose);
        Logger.recordOutput(base + "Pose3d", obs.robotPose());
        Logger.recordOutput(base + "Timestamp", obs.timestamp());
        Logger.recordOutput(
            base + "SkewVsOdomMs", (obs.timestamp() - lastOdomTimestamp) * 1000.0);
        Logger.recordOutput(base + "TagIds", obs.tagIds());
        Logger.recordOutput(base + "TagCount", obs.tagCount());
        Logger.recordOutput(base + "AvgTagDistance", obs.avgTagDistance());
        Logger.recordOutput(base + "Ambiguity", obs.ambiguity());
        Logger.recordOutput(base + "Accepted", res.isAccepted());
        Logger.recordOutput(base + "Reason", res.reason());
        Logger.recordOutput(base + "AcceptedCount", acceptedCount[i]);
        Logger.recordOutput(base + "RejectedCount", rejectedCount[i]);

        Logger.recordOutput(
            base + "ErrorVsFusedMeters",
            camPose.getTranslation().getDistance(currentEstimate.getTranslation()));
        Logger.recordOutput(
            base + "ErrorVsFusedDegrees",
            camPose.getRotation().minus(currentEstimate.getRotation()).getDegrees());

        if (res.isAccepted()) {
            AcceptedObservation a = res.accepted();
            Logger.recordOutput(base + "StdDevLinear", Math.sqrt(a.varX()));
            Logger.recordOutput(base + "StdDevTheta", Math.sqrt(a.varTheta()));
        }
    }

    private void ingestOdometry(OdomSample s) {
        alliance = s.alliance();
        isOnBump = s.isOnBump();
        lastOdomTimestamp = s.timestamp();

        if (s.configHash() != VisionLinkConfig.CONFIG_HASH) {
            Logger.recordOutput("PoseLinkPi/ConfigMismatch", true);
        }

        if (s.resetSeqnum() > appliedResetSeqnum) {
            estimator.reset(
                s.resetPose().getX(), s.resetPose().getY(), s.resetPose().getRotation().getRadians(),
                PiVisionConstants.ANCHOR_LINEAR_VARIANCE,
                PiVisionConstants.ANCHOR_LINEAR_VARIANCE,
                PiVisionConstants.ANCHOR_ANGULAR_VARIANCE);
            appliedResetSeqnum = s.resetSeqnum();
            lastOdomPose = s.odomPose();
            return;
        }

        if (lastOdomPose == null) {
            lastOdomPose = s.odomPose();
            return;
        }

        // Relative motion since the last sample, in the last pose's frame. This
        // spans any dropped packets correctly (cumulative poses), so no drivetrain
        // kinematics are needed here.
        Pose2d delta = s.odomPose().relativeTo(lastOdomPose);
        estimator.addOdometry(
            s.timestamp(), delta.getX(), delta.getY(), delta.getRotation().getRadians(),
            PiVisionConstants.ODOM_VARIANCE_LINEAR,
            PiVisionConstants.ODOM_VARIANCE_LINEAR,
            PiVisionConstants.ODOM_VARIANCE_ANGULAR);

        lastOdomPose = s.odomPose();
    }

    /** Map the marginal covariance to a 0..1 trust score. */
    private static double quality(NativePoseEstimator.Result r) {
        double trace = r.covXX() + r.covYY() + r.covTheta();
        double q = 1.0 / (1.0 + trace);
        return Math.max(0.0, Math.min(1.0, q));
    }

    @Override
    public void endCompetition() {
        if (estimator != null) estimator.close();
    }
}
