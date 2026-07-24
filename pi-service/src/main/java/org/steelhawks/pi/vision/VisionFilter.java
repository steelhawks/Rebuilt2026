package org.steelhawks.pi.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashSet;
import java.util.Optional;
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

    public static Optional<AcceptedObservation> filter(
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
        // Gate the whole observation if it only sees opposing-alliance tags.
        if (!hasAllowed) return Optional.empty();

        Pose3d pose = obs.robotPose();
        boolean reject =
            obs.tagCount() == 0
                || (obs.tagCount() == 1 && obs.ambiguity() > PiVisionConstants.MAX_AMBIGUITY)
                || Math.abs(pose.getZ()) > PiVisionConstants.MAX_ZERROR
                || PiVisionConstants.outOfBounds(pose.getX(), pose.getY());
        if (reject) return Optional.empty();

        double factor = Math.pow(obs.avgTagDistance(), 2.0) / obs.tagCount();
        if (obs.tagCount() == 1) factor *= 3.0;
        if (!hasHub) factor *= PiVisionConstants.NON_HUB_STDDEV_FACTOR;
        if (PiVisionConstants.outOfBounds(currentEstimate.getX(), currentEstimate.getY())) {
            factor *= 0.3;
        }

        double linear = PiVisionConstants.LINEAR_STD_DEV_BASELINE * factor;
        double angular = PiVisionConstants.ANGULAR_STD_DEV_BASELINE * factor;

        double camFactor = PiVisionConstants.CAMERAS[obs.cameraIndex()].stddevFactor();
        if (isOnBump) {
            linear *= PiVisionConstants.BUMP_STDDEV_FACTOR;
            angular *= PiVisionConstants.BUMP_STDDEV_FACTOR;
        } else {
            linear *= camFactor;
            angular *= camFactor;
        }

        // GTSAM wants variances (stddev squared).
        return Optional.of(new AcceptedObservation(
            obs.timestamp(),
            pose.getX(),
            pose.getY(),
            pose.toPose2d().getRotation().getRadians(),
            linear * linear,
            linear * linear,
            angular * angular));
    }
}
