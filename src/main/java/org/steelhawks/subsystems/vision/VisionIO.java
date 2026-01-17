package org.steelhawks.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.steelhawks.Toggles;

public interface VisionIO {

    /** Represents the angle to a simple target, not used for pose estimation. */
    record TargetObservation(Rotation2d tx, Rotation2d ty, int fiducialId) {}

    /** Represents a robot pose sample used for pose estimation. */
    record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type) {}

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    @AutoLog
    class VisionIOInputs {

        public boolean connected = false;
        public TargetObservation latestTargetObservation =
            new TargetObservation(new Rotation2d(), new Rotation2d(), -1);
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
