package org.steelhawks.subsystems.poselink;

import edu.wpi.first.wpilibj.Timer;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.proto.FusedPoseOutput;
import org.steelhawks.proto.RobotOdomInputs;

/**
 * Real UDP + Protobuf implementation of the vision link. Receives
 * {@code FusedPoseOutput} on a background daemon thread and sends
 * {@code RobotOdomInputs} synchronously from the robot loop.
 */
public class PoseLinkIOUDP implements PoseLinkIO {

    private final DatagramSocket socket;
    private final InetSocketAddress piAddress;
    private volatile boolean bound = false;

    // Latest fully-decoded output from the Pi, plus the seqnum the RIO has
    // already reported to its inputs (so updateInputs can flag "new this cycle").
    private final AtomicReference<FusedPoseOutput> latest = new AtomicReference<>(null);
    private long lastReportedSeqnum = -1;
    private long observedRestarts = 0;

    // Health counters (written on the receive thread, read on the robot loop).
    private final AtomicLong packetsReceived = new AtomicLong(0);
    private final AtomicLong packetsDropped = new AtomicLong(0);
    private final AtomicLong packetsStale = new AtomicLong(0);
    private final AtomicLong packetsSent = new AtomicLong(0);
    private final AtomicLong piRestarts = new AtomicLong(0);
    private volatile double lastRxTimestampWallClock = Double.NEGATIVE_INFINITY;

    // Highest seqnum seen (for gap detection) and highest stored (for staleness).
    private long highestSeqnumSeen = -1;

    public PoseLinkIOUDP() {
        DatagramSocket s = null;
        InetSocketAddress pi = null;
        try {
            s = new DatagramSocket(PoseLinkConstants.RIO_RX_PORT);
            s.setSoTimeout(0); // block in the receive thread
            pi = new InetSocketAddress(
                InetAddress.getByName(PoseLinkConstants.PI_HOST), PoseLinkConstants.PI_RX_PORT);
            bound = true;
        } catch (IOException e) {
            Logger.recordOutput("PoseLink/SocketError", "open: " + e.getMessage());
        }
        this.socket = s;
        this.piAddress = pi;

        if (bound) {
            Thread rx = new Thread(this::receiveLoop, "PoseLink-UDP-RX");
            rx.setDaemon(true);
            rx.start();
        }
    }

    private void receiveLoop() {
        byte[] buf = new byte[2048];
        while (bound && !socket.isClosed()) {
            DatagramPacket dp = new DatagramPacket(buf, buf.length);
            try {
                socket.receive(dp);
                FusedPoseOutput msg =
                    FusedPoseOutput.parseFrom(Arrays.copyOf(dp.getData(), dp.getLength()));
                packetsReceived.incrementAndGet();

                long seq = msg.getSeqnum();
                double now = Timer.getFPGATimestamp();
                FusedPoseOutput prev = latest.get();

                // A restarted Pi counts from 0 again, so its packets look stale
                // forever. Treat a backwards jump as a restart when it is either
                // too large to be reordering, or arrives after the stream has
                // already gone stale (a reordered packet turns up while healthy
                // traffic is still flowing, so it can never satisfy that).
                if (prev != null && seq < prev.getSeqnum()) {
                    long backwards = prev.getSeqnum() - seq;
                    boolean restarted =
                        backwards > PoseLinkConstants.SEQNUM_RESTART_GAP
                            || now - lastRxTimestampWallClock
                                > PoseLinkConstants.STALENESS_TIMEOUT_SEC;
                    if (!restarted) {
                        packetsStale.incrementAndGet();
                        continue;
                    }
                    // Adopt the new sequence space wholesale.
                    piRestarts.incrementAndGet();
                    highestSeqnumSeen = -1;
                    prev = null;
                }

                // Gap detection against the highest seqnum we have ever seen.
                if (highestSeqnumSeen >= 0 && seq > highestSeqnumSeen + 1) {
                    packetsDropped.addAndGet(seq - highestSeqnumSeen - 1);
                }
                if (seq > highestSeqnumSeen) {
                    highestSeqnumSeen = seq;
                }

                // Never store an output older than the one already staged. The
                // final monotonic guard against what was applied lives in
                // RobotState.applyFusedPose; this just avoids regressing latest.
                if (prev != null
                    && (seq <= prev.getSeqnum() || msg.getTimestamp() < prev.getTimestamp())) {
                    packetsStale.incrementAndGet();
                    continue;
                }
                latest.set(msg);
                lastRxTimestampWallClock = now;
            } catch (IOException e) {
                if (bound) {
                    Logger.recordOutput("PoseLink/SocketError", "rx: " + e.getMessage());
                }
            }
        }
    }

    @Override
    public void updateInputs(PoseLinkIOInputs inputs) {
        inputs.linkConnected = bound;
        inputs.packetsReceived = packetsReceived.get();
        inputs.packetsDropped = packetsDropped.get();
        inputs.packetsStale = packetsStale.get();
        inputs.packetsSent = packetsSent.get();
        inputs.piRestarts = piRestarts.get();

        double now = Timer.getFPGATimestamp();
        inputs.secondsSinceLastRx =
            Double.isInfinite(lastRxTimestampWallClock)
                ? Double.POSITIVE_INFINITY
                : now - lastRxTimestampWallClock;

        FusedPoseOutput msg = latest.get();
        if (msg == null) {
            inputs.hasNewOutput = false;
            return;
        }

        // A restart moves the stream into a fresh sequence space, so the previous
        // high-water mark no longer means anything - keeping it would leave
        // hasNewOutput false forever and starve RobotState.
        if (inputs.piRestarts != observedRestarts) {
            observedRestarts = inputs.piRestarts;
            lastReportedSeqnum = -1;
        }

        inputs.hasNewOutput = msg.getSeqnum() > lastReportedSeqnum;
        lastReportedSeqnum = msg.getSeqnum();

        inputs.rxSeqnum = msg.getSeqnum();
        inputs.rxTimestamp = msg.getTimestamp();
        inputs.fusedX = msg.getX();
        inputs.fusedY = msg.getY();
        inputs.fusedThetaRadians = msg.getTheta();
        inputs.qualityScore = msg.getQualityScore();
        inputs.covXX = msg.getCovXx();
        inputs.covYY = msg.getCovYy();
        inputs.covTheta = msg.getCovTheta();
        inputs.rxConfigHash = msg.getConfigHash();
        inputs.solveLatencyMs = msg.getSolveLatencyMs();
        inputs.ackResetSeqnum = msg.getAckResetSeqnum();
        inputs.ackRestartSeqnum = msg.getAckRestartSeqnum();
        inputs.piSessionId = msg.getSessionId();
        inputs.piBuildSha = msg.getPiBuildSha();
    }

    @Override
    public void sendOdom(OdomPacket packet) {
        if (!bound || socket.isClosed()) {
            return;
        }
        RobotOdomInputs out =
            RobotOdomInputs.newBuilder()
                .setSeqnum(packet.seqnum())
                .setTimestamp(packet.timestamp())
                .setOdomX(packet.odomPose().getX())
                .setOdomY(packet.odomPose().getY())
                .setOdomTheta(packet.odomPose().getRotation().getRadians())
                .setIsOnBump(packet.isOnBump())
                .setAlliance(packet.alliance())
                .setConfigHash(packet.configHash())
                .setResetSeqnum(packet.resetSeqnum())
                .setResetX(packet.resetPose().getX())
                .setResetY(packet.resetPose().getY())
                .setResetTheta(packet.resetPose().getRotation().getRadians())
                .setSessionId(packet.sessionId())
                .setRestartSeqnum(packet.restartSeqnum())
                .build();

        byte[] bytes = out.toByteArray();
        try {
            socket.send(new DatagramPacket(bytes, bytes.length, piAddress));
            packetsSent.incrementAndGet();
        } catch (IOException e) {
            Logger.recordOutput("PoseLink/SocketError", "tx: " + e.getMessage());
        }
    }
}
