package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.subsystems.vision.VisionConstants.Factors.ObjFactors;
import org.steelhawks.util.LimelightHelpers;

import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

public class ObjectVisionIOLimelight implements ObjectVisionIO {

    private final String name;
    private final int camIndex;
    private final DoubleSubscriber latencySubscriber;

    private final ObjFactors factors;

    public ObjectVisionIOLimelight(String name, int camIndex) {
        this.name = name;
        this.camIndex = camIndex;
        var table = NetworkTableInstance.getDefault().getTable(name);
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        factors = (ObjFactors) Objects.requireNonNull(VisionConstants.getObjDetectConfig())[camIndex].factors();
    }

    private double[] convertPixelsToAngles(double[] pixelCorners, boolean isY) {
        final double resolutionWidth = factors.getFactors()[3];
        final double resolutionHeight = factors.getFactors()[4];
        final double horizontalFov = factors.getFactors()[1];
        final double verticalFov = factors.getFactors()[2];

        double resolution = isY ? resolutionHeight : resolutionWidth;
        double fov = isY ? verticalFov : horizontalFov;

        double[] angles = new double[pixelCorners.length];
        for (int i = 0; i < pixelCorners.length; i++) {
            // norm to [-1, 1]
            double normalized = (pixelCorners[i] - resolution / 2.0) / (resolution / 2.0);
            if (isY) normalized = -normalized; // y increases downward

            // convert to angle
            angles[i] = normalized * (fov / 2.0);
        }
        return angles;
    }

    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        final double confidence = factors.getFactors()[0];

        inputs.connected =
            ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        List<ObjectObservation> observations = new LinkedList<>();
        LimelightHelpers.RawDetection[] detections =
            LimelightHelpers.getRawDetections(name);

        for (LimelightHelpers.RawDetection detection : detections) {
            double limelightLatencySec = latencySubscriber.get() / 1000.0;
            double timestamp = RobotController.getFPGATime() / 1e6 - limelightLatencySec;
            double[] convertedCornersX = convertPixelsToAngles(
                new double[] {
                    detection.corner0_X, detection.corner1_X,
                    detection.corner2_X, detection.corner3_X}, false);
            double[] convertedCornersY = convertPixelsToAngles(
                new double[] {
                    detection.corner0_Y, detection.corner1_Y,
                    detection.corner2_Y, detection.corner3_Y}, true);
            observations.add(
                new ObjectObservation(
                    camIndex,
                    new DetectionInfo(detection, convertedCornersX, convertedCornersY),
                    confidence,
                    timestamp // timestamp in seconds
                )
            );
        }
        inputs.observations = observations.toArray(new ObjectObservation[0]);
    }

    @Override
    public String getName() {
        return name;
    }
}
