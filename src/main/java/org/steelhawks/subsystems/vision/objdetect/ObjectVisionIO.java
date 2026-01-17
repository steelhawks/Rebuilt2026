package org.steelhawks.subsystems.vision.objdetect;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.steelhawks.util.LimelightHelpers;

public interface ObjectVisionIO {

    record ObjectObservation(
        int camIndex,
        DetectionInfo info,
        double confidence,
        double timestamp
    ) {}

    record DetectionInfo(
        int classId,      // Class of the object
        double area,      // Target area (e.g., ta from Limelight)
        double[] tx,      // Corner X positions
        double[] ty       // Corner Y positions
    ) {
        public DetectionInfo(LimelightHelpers.RawDetection raw, double[] txs, double[] tys) {
            this(
                raw.classId,
                raw.ta,
                txs,
                tys
            );
        }
        public DetectionInfo(LimelightHelpers.RawDetection raw) {
            this(
                raw.classId,
                raw.ta,
                new double[] {raw.corner0_X, raw.corner1_X, raw.corner2_X, raw.corner3_X},
                new double[] {raw.corner0_Y, raw.corner1_Y, raw.corner2_Y, raw.corner3_Y}
            );
        }
        public DetectionInfo(PhotonTrackedTarget target) {
            this(
                target.getDetectedObjectClassID(),
                target.getArea(),
                target.getDetectedCorners().stream()
                    .limit(4)
                    .mapToDouble(c -> c.x)
                    .toArray(),
                target.getDetectedCorners().stream()
                    .limit(4)
                    .mapToDouble(c -> c.y)
                    .toArray()
            );
        }
    }

    @AutoLog
    class ObjectVisionIOInputs {
        public boolean connected = false;
        public ObjectObservation latestTargetObservation =
            new ObjectObservation(0, new DetectionInfo(0, 0.0, new double[0], new double[0]), 0.0, 0.0);
        public ObjectObservation[] observations = new ObjectObservation[0];
    }

    default void updateInputs(ObjectVisionIOInputs inputs) {}
    default String getName() { return ""; }
}
