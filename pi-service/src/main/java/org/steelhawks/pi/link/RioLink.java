package org.steelhawks.pi.link;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.pi.PiVisionConstants;
import org.steelhawks.proto.FusedPoseOutput;
import org.steelhawks.proto.RobotOdomInputs;

/** UDP transport: receives {@code RobotOdomInputs}, sends {@code FusedPoseOutput}. */
public class RioLink {

    private final DatagramSocket socket;
    private final InetSocketAddress rioAddress;
    private final ConcurrentLinkedQueue<OdomSample> inbox = new ConcurrentLinkedQueue<>();
    private volatile boolean running = false;
    private long lastRxSeqnum = -1;
    private long dropped = 0;

    public RioLink() {
        DatagramSocket s = null;
        InetSocketAddress rio = null;
        try {
            s = new DatagramSocket(PiVisionConstants.PI_RX_PORT);
            rio = new InetSocketAddress(
                InetAddress.getByName(PiVisionConstants.RIO_HOST), PiVisionConstants.RIO_RX_PORT);
            running = true;
        } catch (IOException e) {
            Logger.recordOutput("RioLink/SocketError", "open: " + e.getMessage());
        }
        this.socket = s;
        this.rioAddress = rio;

        if (running) {
            Thread rx = new Thread(this::receiveLoop, "RioLink-RX");
            rx.setDaemon(true);
            rx.start();
        }
    }

    private void receiveLoop() {
        byte[] buf = new byte[2048];
        while (running && !socket.isClosed()) {
            DatagramPacket dp = new DatagramPacket(buf, buf.length);
            try {
                socket.receive(dp);
                RobotOdomInputs msg =
                    RobotOdomInputs.parseFrom(Arrays.copyOf(dp.getData(), dp.getLength()));
                if (lastRxSeqnum >= 0 && msg.getSeqnum() > lastRxSeqnum + 1) {
                    dropped += msg.getSeqnum() - lastRxSeqnum - 1;
                }
                lastRxSeqnum = Math.max(lastRxSeqnum, msg.getSeqnum());
                inbox.add(decode(msg));
            } catch (IOException e) {
                if (running) Logger.recordOutput("RioLink/SocketError", "rx: " + e.getMessage());
            }
        }
    }

    private static OdomSample decode(RobotOdomInputs msg) {
        return new OdomSample(
            msg.getSeqnum(),
            msg.getTimestamp(),
            new Pose2d(msg.getOdomX(), msg.getOdomY(), new Rotation2d(msg.getOdomTheta())),
            msg.getIsOnBump(),
            msg.getAlliance(),
            msg.getConfigHash(),
            msg.getResetSeqnum(),
            new Pose2d(msg.getResetX(), msg.getResetY(), new Rotation2d(msg.getResetTheta())));
    }

    /** All odom samples received since the last call, in arrival order. */
    public List<OdomSample> drain() {
        List<OdomSample> out = new ArrayList<>();
        OdomSample s;
        while ((s = inbox.poll()) != null) out.add(s);
        return out;
    }

    public boolean isConnected() {
        return running;
    }

    public long droppedCount() {
        return dropped;
    }

    public void sendFusedPose(
        long seqnum, double timestamp, Pose2d pose, double qualityScore,
        double covXX, double covYY, double covTheta, long configHash,
        double solveLatencyMs, long ackResetSeqnum) {
        if (!running || socket.isClosed()) return;
        FusedPoseOutput out = FusedPoseOutput.newBuilder()
            .setSeqnum(seqnum)
            .setTimestamp(timestamp)
            .setX(pose.getX())
            .setY(pose.getY())
            .setTheta(pose.getRotation().getRadians())
            .setQualityScore(qualityScore)
            .setCovXx(covXX)
            .setCovYy(covYY)
            .setCovTheta(covTheta)
            .setConfigHash(configHash)
            .setSolveLatencyMs(solveLatencyMs)
            .setAckResetSeqnum(ackResetSeqnum)
            .build();
        byte[] bytes = out.toByteArray();
        try {
            socket.send(new DatagramPacket(bytes, bytes.length, rioAddress));
        } catch (IOException e) {
            Logger.recordOutput("RioLink/SocketError", "tx: " + e.getMessage());
        }
    }
}
