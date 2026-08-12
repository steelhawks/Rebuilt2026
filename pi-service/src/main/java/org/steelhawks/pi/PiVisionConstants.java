package org.steelhawks.pi;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * Pi-owned vision + fusion configuration. Camera extrinsics and the rejection /
 * stddev tuning live here (not shared with the RIO, per the design). The tag
 * sets and config hash come from the shared {@code VisionLinkConfig}.
 *
 * <p>The camera transforms and module geometry MUST match the physical robot;
 * they are ported from the RIO's old VisionConstants / TunerConstants. Keep them
 * in sync when the hardware changes.
 */
public final class PiVisionConstants {

    private PiVisionConstants() {}

    /**
     * Before enabling this please make sure you know what you are doing
     *
     * <p>Enabling this will cause conflicts between the OrangePi tables for topics such as PowerDistribution/ </p>
     */
    public static final boolean ALLOW_PUBLISH_NT4_TELEMETRY = false;

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Fixed-lag smoother window.
    public static final double LAG_SECONDS = 1.5;

    /**
     * Stop publishing a fused pose once odometry has been silent this long.
     *
     * <p>Without this the service keeps solving and sending at 50 Hz off a frozen
     * graph: in the 2026-06-06 log the RIO link went quiet at t=904 s and the Pi
     * transmitted for another 22 minutes, with the pose still wandering because
     * vision priors kept piling onto the one remaining node with no odometry to
     * constrain them. The RIO's staleness check is on UDP receive time, so it had
     * no way to tell - it saw a steady 50 Hz stream and trusted it.
     */
    public static final double MAX_ODOM_AGE_SEC = 0.25;

    // Rejection thresholds (ported from VisionConstants).
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_ZERROR = 0.75;

    // Stddev baselines, before the range/tag-count/camera factors in VisionFilter.
    //
    // LINEAR was 0.1, which combined with the factors gave 1.84 m at a typical
    // 1.75 m single-tag frame. It never actually ran at that: isOnBump was stuck
    // true (Swerve read ~179 deg of roll from an inverted Pigeon), which took the
    // bump branch and skipped the x2 per-camera factor, so the value really in
    // use was 0.276 m. 0.015 reproduces that 0.276 m now that both bugs are
    // fixed - deliberately holding position trust where the 2026-08-10 logs ran
    // it, so the angular change below is tested on its own rather than
    // confounded with a 6.7x loosening of position.
    public static final double LINEAR_STD_DEV_BASELINE = 0.015;

    /**
     * Angular baseline, in radians, against a factor that is now LINEAR in range
     * (see VisionFilter). 0.3 against the old quadratic factor produced 0.827 rad
     * - 47 deg - of heading stddev on every frame in the 2026-08-10 logs, which
     * is no constraint at all next to odometry at 1e-6 rad^2 per step.
     *
     * <p>0.02 gives roughly 12 deg for a 1.75 m single-tag frame and 2 deg for a
     * two-tag one. Both need a field pass to confirm; the point of this value is
     * that heading is now observable from vision at all.
     */
    public static final double ANGULAR_STD_DEV_BASELINE = 0.02;
    public static final double NON_HUB_STDDEV_FACTOR = 2.0;
    public static final double BUMP_STDDEV_FACTOR = 0.3;

    // Anchor prior strength when a reset/tag first seeds the graph.
    public static final double ANCHOR_LINEAR_VARIANCE = 0.01;
    public static final double ANCHOR_ANGULAR_VARIANCE = 0.02;

    // Per-step wheel-odometry variances (BetweenFactor noise). These are PER
    // ODOMETRY SAMPLE (~20 ms), not per second, so they have to describe how much
    // a single 20 ms step can be wrong - not how far odometry drifts over a match.
    //
    // The old values (2e-3) claimed 4.5 cm of 1-sigma position error and 2.5 deg of
    // heading error in every 20 ms step. Real swerve odometry over 20 ms is
    // sub-millimetre and the Pigeon is a small fraction of a degree, so the graph
    // was under-trusting odometry ~50x and the gyro far more. That makes the
    // between-factor chain floppy: the smoother can bend the whole 1.5 s window to
    // chase a single tag observation, which shows up as jitter.
    //
    // 1e-5 m^2 = 3.2 mm per step (~2.7 cm accumulated over the 1.5 s lag window);
    // 1e-6 rad^2 = 0.06 deg per step (~0.5 deg over the window).
    public static final double ODOM_VARIANCE_LINEAR = 1e-5;
    public static final double ODOM_VARIANCE_ANGULAR = 1e-6;

    /** A camera and its robot->camera transform, plus a per-camera trust factor. */
    public record Cam(String name, Transform3d robotToCamera, double stddevFactor) {}

    // The AprilTag cameras, ported from the RIO's old OMEGA_CAMERA_CONFIG (the Pi
    // now owns these extrinsics). Trim/tune per the physical robot.
    public static final Cam[] CAMERAS = {
        new Cam(
            "ov2311-west",
            new Transform3d(
                Units.inchesToMeters(-10.207),
                Units.inchesToMeters(12.728),
                Units.inchesToMeters(20.679),
                new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(90.0))),
            2.0),
        new Cam(
            "ov2311-northeast",
            new Transform3d(
                Units.inchesToMeters(-11.639),
                Units.inchesToMeters(10.669),
                Units.inchesToMeters(20.774),
                new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-45.0))),
            2.0),
        new Cam(
            "ov9281-east",
            new Transform3d(
                Units.inchesToMeters(-10.207),
                Units.inchesToMeters(-12.716),
                Units.inchesToMeters(20.679),
                new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-90.0))),
            2.0),
        new Cam(
            "ov2311-northwest",
            new Transform3d(
                Units.inchesToMeters(-11.669),
                Units.inchesToMeters(-10.676),
                Units.inchesToMeters(20.773),
                new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(45.0))),
            2.0),
        new Cam(
            "ov9281-southeast",
            new Transform3d(
                Units.inchesToMeters(-13.333),
                Units.inchesToMeters(-12.339),
                Units.inchesToMeters(20.774),
                new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-135.0))),
            2.0),
    };

    // ---- Networking ----
    /** Team number, used to point the NT4 client at the RIO's NT server. */
    public static final int TEAM_NUMBER = 2601;
    public static final int PI_RX_PORT = 5812;
    /** RIO host + UDP port for FusedPoseOutput. */
    public static final String RIO_HOST = "10.26.1.2";
    public static final int RIO_RX_PORT = 5811;

    public static boolean outOfBounds(double x, double y) {
        return x < 0.0
            || x > APRIL_TAG_LAYOUT.getFieldLength()
            || y < 0.0
            || y > APRIL_TAG_LAYOUT.getFieldWidth();
    }
}
