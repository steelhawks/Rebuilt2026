package org.steelhawks.subsystems.poselink;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * Simulation stand-in for the Pi. There is no real GTSAM service in sim, so we
 * synthesize a {@code FusedPoseOutput} from the maple-sim ground-truth pose.
 * This keeps the RIO-side link path (staleness/fallback, monotonic guard,
 * config-hash check) exercised in sim and keeps AdvantageScope replays useful.
 */
public class PoseLinkIOSim implements PoseLinkIO {

    private final Supplier<Pose2d> groundTruthPose;
    private long seqnum = 0;

    public PoseLinkIOSim(Supplier<Pose2d> groundTruthPose) {
        this.groundTruthPose = groundTruthPose;
    }

    @Override
    public void updateInputs(PoseLinkIOInputs inputs) {
        Pose2d pose = groundTruthPose.get();
        seqnum++;

        inputs.linkConnected = true;
        inputs.hasNewOutput = true;
        inputs.rxSeqnum = seqnum;
        inputs.rxTimestamp = Timer.getFPGATimestamp();
        inputs.fusedX = pose.getX();
        inputs.fusedY = pose.getY();
        inputs.fusedThetaRadians = pose.getRotation().getRadians();
        inputs.qualityScore = 1.0;
        inputs.covXX = 0.0;
        inputs.covYY = 0.0;
        inputs.covTheta = 0.0;
        inputs.rxConfigHash = PoseLinkConstants.CONFIG_HASH;
        inputs.solveLatencyMs = 0.0;
        inputs.ackResetSeqnum = -1;

        inputs.packetsReceived = seqnum;
        inputs.packetsSent = seqnum;
        inputs.packetsDropped = 0;
        inputs.packetsStale = 0;
        inputs.secondsSinceLastRx = 0.0;
    }

    // sendOdom is a no-op in sim (default).
}
