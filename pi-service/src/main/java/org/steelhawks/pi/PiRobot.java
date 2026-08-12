package org.steelhawks.pi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;
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
    private double lastOdomRxSeconds = Double.NEGATIVE_INFINITY;

    // The RIO's per-boot session id, echoed back and logged so this log can be
    // paired with the RIO's. Zero until the first packet arrives. A *change*
    // means the RIO restarted under us - see adoptRioSession.
    private long sessionId = 0;
    private long rioSessionChanges = 0;

    // Dashboard-driven restart. -1 means "no baseline yet" - see adoptRestartSeqnum.
    private long appliedRestartSeqnum = -1;
    private boolean restartRequested = false;

    // Residual gate state. False means "do not gate on currentEstimate", either
    // because there is not one yet or because it has lost an argument with too
    // many frames in a row - see noteResidualOutcome.
    private boolean estimateTrusted = false;
    private int consecutiveResidualRejects = 0;

    @Override
    public void robotInit() {
        Logger.recordMetadata("Service", "poselink-pi");
        Logger.recordMetadata("ConfigHash", Long.toString(VisionLinkConfig.CONFIG_HASH));
        BuildInfo.record();
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
            VisionFilter.Result res =
                VisionFilter.filter(obs, alliance, isOnBump, currentEstimate, estimateTrusted);
            noteResidualOutcome(res);
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

        // A graph that has stopped being fed odometry keeps producing a pose, but
        // it is no longer an estimate of where the robot is now - so withhold it
        // rather than let the RIO mistake a steady packet rate for freshness.
        double odomAgeSec = (Logger.getTimestamp() / 1.0e6) - lastOdomRxSeconds;
        boolean odomStale = odomAgeSec > PiVisionConstants.MAX_ODOM_AGE_SEC;

        NativePoseEstimator.Result r = estimator.getResult();
        if (status == NativePoseEstimator.STATUS_OK) {
            currentEstimate = new Pose2d(r.x(), r.y(), new Rotation2d(r.theta()));
            if (!odomStale) {
                link.sendFusedPose(
                    txSeqnum++,
                    lastOdomTimestamp,
                    currentEstimate,
                    quality(r),
                    r.covXX(), r.covYY(), r.covTheta(),
                    VisionLinkConfig.CONFIG_HASH,
                    solveMs,
                    appliedResetSeqnum,
                    sessionId,
                    appliedRestartSeqnum);
            }
        }

        // Pairing + clock alignment for tools/pull_logs.py. SessionId says which
        // RIO log this one belongs to; RioTimestamp is the RIO clock as of the
        // newest odometry packet, which against this log's own timestamps gives
        // the offset between the two logs' time bases. Neither can be recovered
        // afterwards - the Pi has no RTC and both logs start at their own zero.
        Logger.recordOutput("PoseLinkPi/SessionId", String.format("%016x", sessionId));
        Logger.recordOutput("PoseLinkPi/RioTimestamp", lastOdomTimestamp);
        // Nonzero means the RIO restarted while this process stayed up. Expected
        // on a redeploy; if it climbs during a match, the link is flapping.
        Logger.recordOutput("PoseLinkPi/RioSessionChanges", rioSessionChanges);

        Logger.recordOutput("PoseLinkPi/SecondsSinceLastOdom", odomAgeSec);
        Logger.recordOutput("PoseLinkPi/OdomStale", odomStale);
        Logger.recordOutput("PoseLinkPi/Status", status);
        Logger.recordOutput("PoseLinkPi/FusedPose", currentEstimate);
        Logger.recordOutput("PoseLinkPi/SolveMs", solveMs);
        Logger.recordOutput("PoseLinkPi/Nodes", r.nodeCount());
        Logger.recordOutput("PoseLinkPi/Factors", r.factorCount());
        Logger.recordOutput("PoseLinkPi/DroppedOdom", link.droppedCount());
        Logger.recordOutput("PoseLinkPi/CamerasConnected", photon.allConnected());
        Logger.recordOutput("PoseLinkPi/AckRestartSeqnum", appliedRestartSeqnum);
        // If EstimateTrusted sits false, the residual gate has given up and every
        // frame is going in unchecked - the estimate and the cameras are not
        // agreeing on anything.
        Logger.recordOutput("PoseLinkPi/EstimateTrusted", estimateTrusted);
        Logger.recordOutput("PoseLinkPi/ConsecutiveResidualRejects", consecutiveResidualRejects);
        Logger.recordOutput("PoseLinkPi/RestartRequested", restartRequested);

        // Honour a restart last, after this cycle's outputs are recorded, so the
        // log ends with the reason it ended. endCompetition breaks the loop in
        // LoggedRobot.startCompetition, which then calls Logger.end() and flushes
        // the wpilog - so the exit is clean and the file is not truncated. The
        // systemd unit is Restart=always, so exiting IS the restart.
        if (restartRequested) {
            Logger.recordOutput("PoseLinkPi/ExitReason", "restart commanded by RIO");
            endCompetition();
        }
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
        // A new session id means the RIO restarted - redeploy or reboot - while
        // this process kept running. Everything below carries state across
        // samples that is only meaningful within a single RIO boot.
        if (s.sessionId() != 0 && s.sessionId() != sessionId) {
            adoptRioSession();
        }

        adoptRestartSeqnum(s.restartSeqnum());

        alliance = s.alliance();
        isOnBump = s.isOnBump();
        lastOdomTimestamp = s.timestamp();
        lastOdomRxSeconds = Logger.getTimestamp() / 1.0e6;
        sessionId = s.sessionId();

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

    /**
     * Decide whether the residual gate should keep gating.
     *
     * <p>The gate measures each frame against our own estimate, so it is only
     * trustworthy while that estimate is. If enough frames in a row disagree,
     * the cameras outnumber us: stand the gate down so vision can re-anchor the
     * graph, rather than defending a wrong pose against every correction for it.
     */
    private void noteResidualOutcome(VisionFilter.Result res) {
        if (res.reason() == VisionFilter.Reason.RESIDUAL_TOO_LARGE) {
            consecutiveResidualRejects++;
            if (consecutiveResidualRejects >= PiVisionConstants.MAX_CONSECUTIVE_RESIDUAL_REJECTS) {
                estimateTrusted = false;
            }
        } else if (res.isAccepted()) {
            // An accepted frame means the estimate and the cameras agree, so it
            // is worth defending again.
            consecutiveResidualRejects = 0;
            estimateTrusted = PiVisionConstants.ENABLE_RESIDUAL_GATE;
        }
    }

    /**
     * Track the RIO's restart counter, and flag an exit when it advances.
     *
     * <p>The first value seen is <em>adopted, not acted on</em>. This process has
     * by definition already satisfied whatever request was outstanding when it
     * started - it is the result of it. Acting on the baseline instead would exit
     * immediately, systemd would restart us two seconds later into the same
     * pending seqnum, and we would exit again: a restart loop that only ends when
     * the RIO's code restarts. The same reasoning applies on a session change,
     * which is why {@link #adoptRioSession} clears the baseline rather than
     * keeping it - a robot-code redeploy must never restart vision.
     */
    private void adoptRestartSeqnum(long seqnum) {
        if (appliedRestartSeqnum < 0) {
            appliedRestartSeqnum = seqnum;
        } else if (seqnum > appliedRestartSeqnum) {
            appliedRestartSeqnum = seqnum;
            restartRequested = true;
        }
    }

    /**
     * Return every piece of cross-boot state to its just-started value, so a RIO
     * restart leaves this service in the same shape a freshly launched one would
     * be in. The RIO already handles the mirror case (a Pi restart shows up there
     * as a backwards seqnum); this is the missing other half.
     *
     * <p>Left uncorrected, a RIO redeploy corrupts the graph three ways, none of
     * which raise an alert on either side:
     *
     * <ul>
     *   <li>{@code lastOdomPose} still holds the previous boot's cumulative pose,
     *       so the first sample of the new session differences into a
     *       teleport-sized between-factor - injected at {@code
     *       ODOM_VARIANCE_LINEAR}, i.e. asserted to ~3 mm.
     *   <li>{@code appliedResetSeqnum} still holds the previous boot's high-water
     *       mark while {@code RobotState.resetRequestSeqnum} restarts at 0, so
     *       every reset up to that mark is silently dropped and the graph stays
     *       anchored to the old session's origin.
     *   <li>the graph keeps the old boot's nodes and factors.
     * </ul>
     *
     * <p>Dropping {@code appliedResetSeqnum} to -1 is what re-anchors it: the
     * same packet that triggered this carries {@code resetSeqnum >= 0}, so it
     * falls through to the reset branch in {@link #ingestOdometry} and rebuilds
     * the graph around the RIO's reset pose before any odometry is applied.
     */
    private void adoptRioSession() {
        rioSessionChanges++;
        lastOdomPose = null;
        appliedResetSeqnum = -1;
        appliedRestartSeqnum = -1;
        currentEstimate = new Pose2d();
        // The graph is about to be re-anchored, so the old estimate is not
        // something to gate incoming frames against.
        estimateTrusted = false;
        consecutiveResidualRejects = 0;
        Arrays.fill(acceptedCount, 0);
        Arrays.fill(rejectedCount, 0);
    }

    /** Map the marginal covariance to a 0..1 trust score. */
    private static double quality(NativePoseEstimator.Result r) {
        double trace = r.covXX() + r.covYY() + r.covTheta();
        double q = 1.0 / (1.0 + trace);
        return Math.max(0.0, Math.min(1.0, q));
    }

    @Override
    public void endCompetition() {
        super.endCompetition();
        if (estimator != null) {
            estimator.close();
            // A use after this point is a bug; take an NPE over a segfault.
            estimator = null;
        }
    }
}
