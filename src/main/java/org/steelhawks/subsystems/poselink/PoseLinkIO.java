package org.steelhawks.subsystems.poselink;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.steelhawks.proto.AllianceColor;

/**
 * IO boundary for the vision/pose link to the Orange Pi.
 *
 * <p>Everything the RIO <em>receives</em> from the Pi lives in
 * {@link PoseLinkIOInputs} so it is captured by AdvantageKit and replayable
 * offline. The <em>send</em> side ({@link #sendOdom}) is a live side effect and
 * is a no-op during replay.
 */
public interface PoseLinkIO {

    /**
     * Everything the RIO packages up and ships to the Pi in one
     * {@code RobotOdomInputs} packet. Assembled by {@link PoseLink} from
     * {@link org.steelhawks.RobotState}.
     */
    record OdomPacket(
        long seqnum,
        double timestamp,
        Pose2d odomPose,
        boolean isOnBump,
        AllianceColor alliance,
        long configHash,
        long resetSeqnum,
        Pose2d resetPose,
        long sessionId) {}

    @AutoLog
    class PoseLinkIOInputs {
        /** The RIO's receive socket is bound and listening. */
        public boolean linkConnected = false;

        /** A {@code FusedPoseOutput} newer than the last one arrived this cycle. */
        public boolean hasNewOutput = false;

        // ---- Latest decoded FusedPoseOutput ----
        public long rxSeqnum = -1;
        public double rxTimestamp = 0.0;
        public double fusedX = 0.0;
        public double fusedY = 0.0;
        public double fusedThetaRadians = 0.0;
        public double qualityScore = 0.0;
        public double covXX = 0.0;
        public double covYY = 0.0;
        public double covTheta = 0.0;
        public long rxConfigHash = 0;
        public double solveLatencyMs = 0.0;
        public long ackResetSeqnum = -1;
        /** Session id the Pi is echoing back; 0 until it has heard from this RIO. */
        public long piSessionId = 0;
        /** Short git SHA of the Pi service build, or empty if it did not stamp one. */
        public String piBuildSha = "";

        // ---- Link health counters ----
        public long packetsSent = 0;
        public long packetsReceived = 0;
        /** Count of seqnum gaps observed (dropped/reordered packets). */
        public long packetsDropped = 0;
        /** Count of received packets discarded for being older than the last applied. */
        public long packetsStale = 0;
        /** Times the Pi's seqnum restarted, i.e. the Pi service restarted. */
        public long piRestarts = 0;
        /** Wall-clock seconds since the last valid FusedPoseOutput was received. */
        public double secondsSinceLastRx = Double.POSITIVE_INFINITY;
    }

    /** Copy the latest received state into {@code inputs}. */
    default void updateInputs(PoseLinkIOInputs inputs) {}

    /** Serialize and send one odometry packet to the Pi. No-op during replay. */
    default void sendOdom(OdomPacket packet) {}
}
