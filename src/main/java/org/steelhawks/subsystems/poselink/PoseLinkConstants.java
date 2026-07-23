package org.steelhawks.subsystems.poselink;

import java.util.Arrays;
import java.util.Objects;
import org.steelhawks.subsystems.vision.VisionConstants;

/**
 * Configuration for the RIO side of the vision/pose-estimation link to the
 * Orange Pi. See {@code src/main/proto/vision_link.proto} and the package
 * README for the wire contract.
 */
public final class PoseLinkConstants {

    private PoseLinkConstants() {}

    /**
     * Bump this whenever the wire contract or the tag/layout config below
     * changes in a way the Pi must know about. Combined into {@link #CONFIG_HASH}.
     */
    public static final int CONFIG_VERSION = 1;

    /** Hostname/IP of the Orange Pi vision service on the robot network. */
    public static final String PI_HOST = "10.26.1.11";

    /** UDP port the Pi listens on for {@code RobotOdomInputs}. */
    public static final int PI_RX_PORT = 5810;

    /** UDP port the RIO listens on for {@code FusedPoseOutput}. */
    public static final int RIO_RX_PORT = 5811;

    /**
     * How long a fused pose stays "fresh" before the RIO falls back to its
     * local wheel-only dead-reckoning pose. Start here and tune from testing;
     * the spec suggested 150-250ms. At 50Hz link rate this tolerates ~10 dropped
     * packets before falling back.
     *
     * <p>Mirrored into {@link org.steelhawks.RobotState#FUSED_STALENESS_TIMEOUT_SEC}
     * which owns the actual fresh/stale decision.
     */
    public static final double STALENESS_TIMEOUT_SEC = 0.2;

    /** Link send rate. Independent of the RIO's main loop rate. */
    public static final double LINK_PERIOD_SEC = 0.02;

    /**
     * Hash of the RIO-side vision config the Pi must agree on: the config
     * version plus the alliance tag sets and hub-tag set that drive
     * whitelisting. If the Pi computes a different value, poses are logged and
     * flagged rather than silently trusted.
     */
    public static final long CONFIG_HASH = computeConfigHash();

    private static long computeConfigHash() {
        return ((long) Objects.hash(
                CONFIG_VERSION,
                Arrays.hashCode(VisionConstants.BLUE_TAGS),
                Arrays.hashCode(VisionConstants.RED_TAGS),
                Arrays.hashCode(VisionConstants.BLUE_HUB_ONLY),
                Arrays.hashCode(VisionConstants.RED_HUB_ONLY),
                VisionConstants.HUB_TAG_IDS.hashCode()))
            & 0xFFFFFFFFL;
    }
}
