package org.steelhawks.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

import java.util.function.Supplier;

/**
 * A trigger that checks if the robot is within a specified bounding box on the field.
 * The bounding box is defined by its minimum and maximum X and Y coordinates.
 * The coordinates are adjusted based on the alliance color.
 *
 * @author Farhan Jamil
 */
public class FieldBoundingBox extends Trigger {

    /**
     * Creates a new FieldBoundingBox trigger.
     *
     * @param name Name of the trigger for logging purposes.
     * @param startPoint The starting point of the bounding box.
     * @param endPoint The ending point of the bounding box.
     * @param robotPose Supplier for the robot's current pose.
     */
    public FieldBoundingBox(
        String name,
        Translation2d startPoint,
        Translation2d endPoint,
        Supplier<Pose2d> robotPose
    ) {
        this(name, startPoint.getX(), endPoint.getX(), startPoint.getY(), endPoint.getY(), robotPose);
    }

    /**
     * Creates a new FieldBoundingBox trigger.
     *
     * @param name Name of the trigger for logging purposes.
     * @param minX Minimum X coordinate of the bounding box.
     * @param maxX Maximum X coordinate of the bounding box.
     * @param minY Minimum Y coordinate of the bounding box.
     * @param maxY Maximum Y coordinate of the bounding box.
     * @param robotPose Supplier for the robot's current pose.
     */
    public FieldBoundingBox(
        String name,
        double minX,
        double maxX,
        double minY,
        double maxY,
        Supplier<Pose2d> robotPose
    ) {
        super(() -> {
            double dynamicMinX = AllianceFlip.applyX(minX);
            double dynamicMaxX = AllianceFlip.applyX(maxX);
            double dynamicMinY = AllianceFlip.applyY(minY);
            double dynamicMaxY = AllianceFlip.applyY(maxY);

            return (robotPose.get().getX() >= dynamicMinX && robotPose.get().getX() <= dynamicMaxX)
                && (robotPose.get().getY() >= dynamicMinY && robotPose.get().getY() <= dynamicMaxY);
        });

        if (Constants.getRobot() != Constants.RobotType.SIMBOT)
            return;

        new VirtualSubsystem("FieldBoundingBox/" + name) {
            @Override
            public void periodic() {
                double dynamicMinX = AllianceFlip.applyX(minX);
                double dynamicMaxX = AllianceFlip.applyX(maxX);
                double dynamicMinY = AllianceFlip.applyY(minY);
                double dynamicMaxY = AllianceFlip.applyY(maxY);

                Logger.recordOutput("FieldBoundingBox/" + name + "/MinX", dynamicMinX);
                Logger.recordOutput("FieldBoundingBox/" + name + "/MaxX", dynamicMaxX);
                Logger.recordOutput("FieldBoundingBox/" + name + "/MinY", dynamicMinY);
                Logger.recordOutput("FieldBoundingBox/" + name + "/MaxY", dynamicMaxY);

                Logger.recordOutput("FieldBoundingBox/" + name + "/Box",
                    new Pose2d(dynamicMinX, dynamicMinY, new Rotation2d()),
                    new Pose2d(dynamicMaxX, dynamicMinY, new Rotation2d()),
                    new Pose2d(dynamicMinX, dynamicMaxY, new Rotation2d()),
                    new Pose2d(dynamicMaxX, dynamicMaxY, new Rotation2d()));
            }
        };
    }
}
