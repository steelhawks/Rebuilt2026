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

    /**
     * UDP port the Pi listens on for {@code RobotOdomInputs}.
     *
     * <p>Deliberately not 5810: photonlib reserves that for its
     * {@code TimeSyncServer}, which runs inside the Pi service process. See the
     * comment on {@code PiVisionConstants.PI_RX_PORT} - the two must match, and
     * changing one without redeploying the other kills the link.
     */
    public static final int PI_RX_PORT = 5812;

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
     * How far outside the field a fused pose may sit before the RIO refuses it,
     * in metres.
     *
     * <p>Until this existed the RIO applied whatever the Pi sent. In the
     * 2026-08-12 session e4d44f66 the graph diverged into a growing oscillation -
     * 63 m, then -96 m, then 138 m, ending at 2.25e8 m with a covariance of
     * 2.4e10 - and every one of those poses went straight into
     * {@link org.steelhawks.RobotState#applyFusedPose}. The Pi reported
     * STATUS_OK throughout, so nothing upstream caught it either.
     *
     * <p>The margin is generous on purpose: a real pose can sit slightly off the
     * field when the robot is against the wall, and this is a last-resort guard
     * against nonsense, not a tuning knob.
     */
    public static final double MAX_POSE_OUT_OF_BOUNDS_METERS = 1.0;

    /**
     * Reject a fused pose whose quality score is below this.
     *
     * <p>{@code quality = 1 / (1 + trace(covariance))}, so a diverging graph
     * drives it to essentially zero - it was ~4e-11 at the worst point of
     * e4d44f66. Set low deliberately: this catches numerical garbage, not a pose
     * that is merely uncertain. Normal operation sits at 0.7 to 0.99, and a
     * legitimately unsure estimate is still far better than dead reckoning.
     */
    public static final double MIN_FUSED_QUALITY = 0.02;

    /**
     * Fingerprint of the config the Pi must agree on (tag sets + version),
     * computed from the shared {@link VisionLinkConfig}. If the Pi reports a
     * different value, poses are logged and flagged rather than silently trusted.
     */
    public static final long CONFIG_HASH = VisionLinkConfig.CONFIG_HASH;
}
