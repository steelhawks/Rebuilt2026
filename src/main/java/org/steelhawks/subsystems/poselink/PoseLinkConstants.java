package org.steelhawks.subsystems.poselink;

import org.steelhawks.common.VisionLinkConfig;

/**
 * Configuration for the RIO side of the vision/pose-estimation link to the
 * Orange Pi. See {@code common/src/main/proto/vision_link.proto} and the package
 * README for the wire contract. The tag sets / config-hash inputs live in the
 * shared {@link VisionLinkConfig} so the RIO and Pi can never disagree by value.
 */
public final class PoseLinkConstants {

    private PoseLinkConstants() {}

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
     * A backwards seqnum jump larger than this means the Pi restarted, not that a
     * packet was reordered.
     *
     * <p>One second of link traffic. Real UDP reordering on the robot LAN is a
     * handful of packets at most, so this cannot be tripped by a late arrival.
     */
    public static final long SEQNUM_RESTART_GAP = 50;

    /**
     * Fingerprint of the config the Pi must agree on (tag sets + version),
     * computed from the shared {@link VisionLinkConfig}. If the Pi reports a
     * different value, poses are logged and flagged rather than silently trusted.
     */
    public static final long CONFIG_HASH = VisionLinkConfig.CONFIG_HASH;
}
