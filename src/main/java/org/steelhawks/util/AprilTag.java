package org.steelhawks.util;

import edu.wpi.first.math.geometry.Pose3d;
import org.steelhawks.subsystems.vision.VisionConstants;

public record AprilTag(int id, Pose3d pose) {
    @Override
    public Pose3d pose() {
        return VisionConstants.APRIL_TAG_LAYOUT.getTagPose(id).get();
    }

    public static int tagToArrayIndex(int tag) {
        return tag - 1;
    }
}
