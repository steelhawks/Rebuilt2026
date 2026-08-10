package org.steelhawks.subsystems.poselink;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.RobotState;
import org.steelhawks.Subsystems;
import org.steelhawks.proto.AllianceColor;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.Elastic;
import org.steelhawks.util.LogSession;
import org.steelhawks.util.LoopTimeUtil;

/**
 * RIO side of the vision/pose-estimation link to the Orange Pi.
 *
 * <p>Replaces the old {@code Vision} subsystem. Each cycle it ships the robot's
 * wheel odometry + context to the Pi and consumes the Pi's fused pose, feeding
 * it into {@link RobotState} in place of the removed local pose estimator. The
 * Pi now owns PhotonVision, tag whitelisting/rejection, stddev weighting, and
 * the GTSAM iSAM2 factor graph; the RIO keeps a wheel-only dead-reckoning
 * fallback (see {@link RobotState}) for when the link goes stale.
 */
public class PoseLink extends SubsystemBase {

    private final PoseLinkIO io;
    private final PoseLinkIOInputsAutoLogged inputs = new PoseLinkIOInputsAutoLogged();

    private final Alert disconnectedAlert =
        new Alert("PoseLink: no socket to the Orange Pi vision service.", AlertType.kError);
    private final Alert staleAlert =
        new Alert("PoseLink: fused pose stale, running on wheel-only fallback.", AlertType.kWarning);
    private final Alert configMismatchAlert =
        new Alert("PoseLink: Pi config hash != RIO config hash (layout/alliance mismatch).",
            AlertType.kError);

    private final Alert sessionMismatchAlert =
        new Alert("PoseLink: Pi is not reporting this RIO's session id; logs will not pair.",
            AlertType.kWarning);
    private final Alert restartedAlert =
        new Alert("PoseLink: the Pi vision service restarted mid-session.", AlertType.kWarning);

    private long txSeqnum = 0;

    /**
     * Bumped by {@link #restartVisionCommand()}. Rides along on every packet; the
     * Pi acts on a strictly higher value than the one it has adopted, so a single
     * increment survives however many packets get dropped on the way.
     */
    private long restartSeqnum = 0;

    public PoseLink() {
        if (RobotBase.isReal()) {
            io = new PoseLinkIOUDP();
        } else if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
            io = new PoseLinkIOSim(Swerve.getDriveSimulation()::getSimulatedDriveTrainPose);
        } else {
            // Replay or non-sim desktop: no live link. RobotState falls back to
            // wheel-only odometry, which is exactly the intended stale behaviour.
            io = new PoseLinkIO() {};
        }
    }

    @Override
    public void periodic() {
        RobotState rs = RobotState.getInstance();

        // ---- RIO -> Pi: ship the cumulative wheel-odometry pose + context ----
        // The Pi differences consecutive poses into odometry between-factors, so
        // it needs no drivetrain kinematics of its own.
        Optional<RobotState.OdometryObservation> latestOdom = rs.getLatestOdometry();
        if (latestOdom.isPresent()) {
            RobotState.OdometryObservation odom = latestOdom.get();
            io.sendOdom(
                new PoseLinkIO.OdomPacket(
                    txSeqnum++,
                    odom.timestamp(),
                    rs.getWheelOdometryPose(),
                    Subsystems.swerve().isOnBump(),
                    toProto(DriverStation.getAlliance()),
                    PoseLinkConstants.CONFIG_HASH,
                    rs.getResetRequestSeqnum(),
                    rs.getResetRequestPose(),
                    LogSession.id(),
                    restartSeqnum));
        }

        // ---- Pi -> RIO: consume the fused pose ----
        io.updateInputs(inputs);
        Logger.processInputs("PoseLink", inputs);

        disconnectedAlert.set(!inputs.linkConnected);
        restartedAlert.set(inputs.piRestarts > 0);

        if (inputs.hasNewOutput) {
            if (inputs.rxConfigHash != PoseLinkConstants.CONFIG_HASH) {
                // Never silently trust a mismatched config: log loudly, drop the pose.
                configMismatchAlert.set(true);
                Logger.recordOutput("PoseLink/ConfigMismatch", true);
                Logger.recordOutput("PoseLink/RxConfigHash", inputs.rxConfigHash);
            } else {
                configMismatchAlert.set(false);
                Logger.recordOutput("PoseLink/ConfigMismatch", false);
                rs.applyFusedPose(
                    new RobotState.FusedPoseObservation(
                        inputs.rxSeqnum,
                        inputs.rxTimestamp,
                        new Pose2d(inputs.fusedX, inputs.fusedY,
                            new Rotation2d(inputs.fusedThetaRadians)),
                        inputs.qualityScore));
            }
        }

        boolean usingFallback = !rs.isFusedPoseFresh();
        staleAlert.set(usingFallback && inputs.linkConnected);
        Logger.recordOutput("PoseLink/UsingFallback", usingFallback);
        Logger.recordOutput("PoseLink/ConfigHash", PoseLinkConstants.CONFIG_HASH);
        Logger.recordOutput("PoseLink/SecondsSinceLastRx", inputs.secondsSinceLastRx);

        // Log pairing state so this log alone answers "which Pi log goes with this
        // one, and which vision build produced it?". The Pi echoes the session id
        // back; anything other than our own means it has not yet picked up this
        // boot (0 on a fresh Pi) or is still running off a previous one.
        Logger.recordOutput("PoseLink/SessionId", LogSession.idHex());
        Logger.recordOutput("PoseLink/PiSessionId", String.format("%016x", inputs.piSessionId));
        Logger.recordOutput("PoseLink/PiBuildSha", inputs.piBuildSha);
        // Restart handshake. Pending means we have asked and the Pi has not yet
        // come back reporting our number - normal for the few seconds it takes
        // systemd to cycle it, a stuck problem if it never clears.
        Logger.recordOutput("PoseLink/RestartSeqnum", restartSeqnum);
        Logger.recordOutput("PoseLink/AckRestartSeqnum", inputs.ackRestartSeqnum);
        Logger.recordOutput(
            "PoseLink/RestartPending", inputs.ackRestartSeqnum < restartSeqnum);

        boolean paired = inputs.piSessionId == LogSession.id();
        Logger.recordOutput("PoseLink/SessionPaired", paired);
        sessionMismatchAlert.set(inputs.hasNewOutput && !paired);

        LoopTimeUtil.record("PoseLink");
    }

    /**
     * A command that asks the Pi to restart its vision service, for a dashboard
     * button. The Pi exits and systemd (see {@code deploy/poselink.service},
     * {@code Restart=always}, {@code RestartSec=2}) brings it back, so expect the
     * link to go stale for a few seconds and the RIO to run on wheel-only
     * odometry meanwhile - exactly the {@code UsingFallback} path.
     *
     * <p>Runs while disabled on purpose: restarting vision from the pit, between
     * matches, is the whole point. It takes no requirements, so it cannot
     * interrupt anything.
     */
    public Command restartVisionCommand() {
        return Commands.runOnce(() -> {
                restartSeqnum++;
                Elastic.sendNotification(
                    new Elastic.Notification(
                        Elastic.Notification.NotificationLevel.INFO,
                        "Vision restarting",
                        "Asked the Orange Pi to restart its vision service. Pose falls back to "
                            + "wheel-only odometry for a few seconds."));
            })
            .ignoringDisable(true)
            .withName("Restart Vision");
    }

    private static AllianceColor toProto(Optional<Alliance> alliance) {
        if (alliance.isEmpty()) {
            return AllianceColor.ALLIANCE_UNKNOWN;
        }
        return alliance.get() == Alliance.Blue
            ? AllianceColor.ALLIANCE_BLUE
            : AllianceColor.ALLIANCE_RED;
    }
}
