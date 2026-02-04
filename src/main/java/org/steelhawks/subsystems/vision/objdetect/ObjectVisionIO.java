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
        int classId,
        double area,
        double tx0, double ty0,
        double tx1, double ty1,
        double tx2, double ty2,
        double tx3, double ty3
    ) {
        public DetectionInfo(LimelightHelpers.RawDetection raw) {
            this(
                raw.classId,
                raw.ta,
                raw.corner0_X, raw.corner0_Y,
                raw.corner1_X, raw.corner1_Y,
                raw.corner2_X, raw.corner2_Y,
                raw.corner3_X, raw.corner3_Y
            );
        }

        public DetectionInfo(PhotonTrackedTarget target) {
            this(
                target.getDetectedObjectClassID(),
                target.getArea(),
                target.getDetectedCorners().get(0).x,
                target.getDetectedCorners().get(0).y,
                target.getDetectedCorners().get(1).x,
                target.getDetectedCorners().get(1).y,
                target.getDetectedCorners().get(2).x,
                target.getDetectedCorners().get(2).y,
                target.getDetectedCorners().get(3).x,
                target.getDetectedCorners().get(3).y
            );
        }

        public double[] tx() {
            return new double[] { tx0, tx1, tx2, tx3 };
        }

        public double[] ty() {
            return new double[] { ty0, ty1, ty2, ty3 };
        }
    }

    @AutoLog
    class ObjectVisionIOInputs {
        public boolean connected = false;
        public ObjectObservation[] observations = new ObjectObservation[0];
    }

    default void updateInputs(ObjectVisionIOInputs inputs) {}
    default String getName() { return ""; }
}
