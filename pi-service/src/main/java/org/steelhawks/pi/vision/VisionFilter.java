package org.steelhawks.pi.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashSet;
import java.util.Set;
import org.steelhawks.common.VisionLinkConfig;
import org.steelhawks.pi.PiVisionConstants;
import org.steelhawks.proto.AllianceColor;

/**
 * Port of the old {@code Vision.java} gating: alliance tag whitelist, pose
 * rejection (ambiguity / Z-error / field bounds), and the distance/tag-count/
 * hub/bump stddev weighting. Pure function of its inputs so it stays replayable.
 */
public final class VisionFilter {

    private VisionFilter() {}

    /** Why an observation was kept or thrown away. Logged per camera. */
    public enum Reason {
        ACCEPTED,
        NO_ALLOWED_TAG,
        NO_TAGS,
        AMBIGUOUS,
        Z_ERROR,
        OUT_OF_BOUNDS
    }

    /**
     * Outcome of filtering one observation. {@code accepted} is null unless
     * {@code reason} is {@link Reason#ACCEPTED}; the reason is kept either way so
     * a rejected frame still leaves a trace in the log.
     */
    public record Result(AcceptedObservation accepted, Reason reason) {

        private static Result reject(Reason reason) {
            return new Result(null, reason);
        }

        public boolean isAccepted() {
            return accepted != null;
        }
    }

    private static final Set<Integer> BLUE = toSet(VisionLinkConfig.BLUE_TAGS);
    private static final Set<Integer> RED = toSet(VisionLinkConfig.RED_TAGS);
    private static final Set<Integer> ALL = toSet(VisionLinkConfig.ALL_ALLOWED_TAGS);

    private static Set<Integer> toSet(int[] ids) {
        Set<Integer> s = new HashSet<>();
        for (int id : ids) s.add(id);
        return s;
    }

    private static Set<Integer> allowedFor(AllianceColor alliance) {
        return switch (alliance) {
            case ALLIANCE_BLUE -> BLUE;
            case ALLIANCE_RED -> RED;
            default -> ALL;
        };
    }

    public static Result filter(
        CameraObservation obs,
        AllianceColor alliance,
        boolean isOnBump,
        Pose2d currentEstimate) {

        Set<Integer> allowed = allowedFor(alliance);
        boolean hasAllowed = false;
        boolean hasHub = false;
        for (int id : obs.tagIds()) {
            if (allowed.contains(id)) hasAllowed = true;
            if (VisionLinkConfig.HUB_TAG_IDS.contains(id)) hasHub = true;
        }
        if (!hasAllowed) return Result.reject(Reason.NO_ALLOWED_TAG);
        Pose3d pose = obs.robotPose();
        if (obs.tagCount() == 0) {
            return Result.reject(Reason.NO_TAGS);
        }
        if (obs.tagCount() == 1 && obs.ambiguity() > PiVisionConstants.MAX_AMBIGUITY) {
            return Result.reject(Reason.AMBIGUOUS);
        }
        if (Math.abs(pose.getZ()) > PiVisionConstants.MAX_ZERROR) {
            return Result.reject(Reason.Z_ERROR);
        }
        if (PiVisionConstants.outOfBounds(pose.getX(), pose.getY())) {
            return Result.reject(Reason.OUT_OF_BOUNDS);
        }

        // Position error from a tag grows with the SQUARE of range: the tag's
        // apparent size shrinks linearly, so a fixed pixel error maps to a
        // depth error that grows quadratically.
        double factor = Math.pow(obs.avgTagDistance(), 2.0) / obs.tagCount();
        if (obs.tagCount() == 1) factor *= 3.0;

        // Heading does NOT degrade quadratically, and sharing the linear factor
        // was making the graph ignore vision heading entirely. In the 2026-08-10
        // logs a 1.75 m single-tag frame produced 0.3 * 9.195 = 2.76 rad of
        // angular stddev - the solver was told "trust this heading to +-158 deg",
        // against odometry asserted to 0.057 deg per step. The fused heading sat
        // ~55 deg off what all three cameras agreed on, from the anchor onward,
        // and no number of tags could pull it back.
        //
        // Scale angle LINEARLY with range instead. The single-tag penalty stays
        // (a lone tag's orientation really is ambiguity-prone); a two-tag frame
        // now yields a genuinely strong heading fix, which is the shape we want.
        double angularFactor = obs.avgTagDistance() / obs.tagCount();
        if (obs.tagCount() == 1) angularFactor *= 3.0;

        if (!hasHub) {
            factor *= PiVisionConstants.NON_HUB_STDDEV_FACTOR;
            angularFactor *= PiVisionConstants.NON_HUB_STDDEV_FACTOR;
        }
        if (PiVisionConstants.outOfBounds(currentEstimate.getX(), currentEstimate.getY())) {
            // Our own estimate is off the field, so it cannot be the sane one.
            factor *= 0.3;
            angularFactor *= 0.3;
        }

        double linear = PiVisionConstants.LINEAR_STD_DEV_BASELINE * factor;
        double angular = PiVisionConstants.ANGULAR_STD_DEV_BASELINE * angularFactor;

        // The per-camera trust factor always applies. It used to be in an else
        // against the bump branch, so a bump silently discarded it - and with
        // isOnBump stuck true (see Swerve.isOnBump) that was every single frame.
        linear *= PiVisionConstants.CAMERAS[obs.cameraIndex()].stddevFactor();
        angular *= PiVisionConstants.CAMERAS[obs.cameraIndex()].stddevFactor();

        // On a bump the wheels are slipping, so odometry is the unreliable one
        // and vision earns extra weight. Now an additional multiplier rather
        // than a replacement.
        if (isOnBump) {
            linear *= PiVisionConstants.BUMP_STDDEV_FACTOR;
            angular *= PiVisionConstants.BUMP_STDDEV_FACTOR;
        }

        // GTSAM wants variances (stddev squared).
        return new Result(
            new AcceptedObservation(
                obs.timestamp(),
                pose.getX(),
                pose.getY(),
                pose.toPose2d().getRotation().getRadians(),
                linear * linear,
                linear * linear,
                angular * angular),
            Reason.ACCEPTED);
    }
}
