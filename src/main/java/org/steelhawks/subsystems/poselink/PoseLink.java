package org.steelhawks.subsystems.poselink;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.RobotState;
import org.steelhawks.Subsystems;
import org.steelhawks.proto.AllianceColor;
import org.steelhawks.subsystems.swerve.Swerve;
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

    private long txSeqnum = 0;

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
                    rs.getResetRequestPose()));
        }

        // ---- Pi -> RIO: consume the fused pose ----
        io.updateInputs(inputs);
        Logger.processInputs("PoseLink", inputs);

        disconnectedAlert.set(!inputs.linkConnected);

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
        LoopTimeUtil.record("PoseLink");
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
