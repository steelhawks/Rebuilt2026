package org.steelhawks.pi;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.steelhawks.common.VisionLinkConfig;
import org.steelhawks.pi.gtsam.NativePoseEstimator;

/**
 * Slice-1 skeleton. Its only job right now is the bring-up smoke test:
 * <ol>
 *   <li>start AdvantageKit logging on the Pi (validates akit's linuxarm64
 *       native — the riskiest infra unknown), and</li>
 *   <li>load {@code libposelink_gtsam.so} and run a native round trip
 *       (validates JNI).</li>
 * </ol>
 *
 * <p>Slice 2+ fills in: UDP {@code RobotOdomInputs} IO, PhotonVision IO,
 * the Java rejection/stddev port, the GTSAM graph, and {@code FusedPoseOutput}
 * emit - all inside this {@code periodic()} loop (design Q12).
 */
public class PiRobot extends LoggedRobot {

    private NativePoseEstimator estimator;
    private long loopCount = 0;

    @Override
    public void robotInit() {
        Logger.recordMetadata("Service", "poselink-pi");
        Logger.recordMetadata("ConfigHash", Long.toString(VisionLinkConfig.CONFIG_HASH));
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.start();

        // ---- JNI + native round-trip smoke test ----
        System.out.println("[poselink] native: " + NativePoseEstimator.version());
        estimator = new NativePoseEstimator(1.5);
        estimator.reset(1.0, 2.0, 0.0, 0.01, 0.01, 0.02);
        estimator.addOdometry(0.02, 0.10, 0.0, 0.0, 0.02, 0.02, 0.02);
        estimator.update();
        NativePoseEstimator.Result r = estimator.getResult();
        System.out.printf(
            "[poselink] round-trip: x=%.3f y=%.3f theta=%.3f status=%d nodes=%d%n",
            r.x(), r.y(), r.theta(), r.status(), r.nodeCount());
    }

    @Override
    public void robotPeriodic() {
        // Heartbeat until the real fusion loop lands in slice 2.
        Logger.recordOutput("PoseLinkPi/LoopCount", ++loopCount);
    }

    @Override
    public void endCompetition() {
        if (estimator != null) {
            estimator.close();
        }
    }
}
