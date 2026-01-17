

package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.dyn4j.geometry.Vector2;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AprilTag;

public class FieldConstants {

    public static final double FIELD_LENGTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldLength();
    public static final double FIELD_WIDTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();

    public static final Field2d FIELD_2D = new Field2d();

    public static final AprilTag[] APRIL_TAGS = {
        new AprilTag(
                1,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(657.37),
                                Units.inchesToMeters(25.80),
                                Units.inchesToMeters(58.50)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(126)))),
        new AprilTag(
                2,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(657.37),
                                Units.inchesToMeters(291.20),
                                Units.inchesToMeters(58.50)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(234)))),
        new AprilTag(
                3,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(455.15),
                                Units.inchesToMeters(317.15),
                                Units.inchesToMeters(51.25)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(270)))),
        new AprilTag(
                4,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(365.20),
                                Units.inchesToMeters(241.64),
                                Units.inchesToMeters(73.54)),
                        new Rotation3d(
                                Units.degreesToRadians(30),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0)))),
        new AprilTag(
                5,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(365.20),
                                Units.inchesToMeters(75.39),
                                Units.inchesToMeters(73.54)),
                        new Rotation3d(
                                Units.degreesToRadians(30),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0)))),
        new AprilTag(
                6,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(530.49),
                                Units.inchesToMeters(130.17),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(300)))),
        new AprilTag(
                7,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(546.87),
                                Units.inchesToMeters(158.50),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0)))),
        new AprilTag(
                8,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(530.49),
                                Units.inchesToMeters(186.83),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(60)))),
        new AprilTag(
                9,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(497.77),
                                Units.inchesToMeters(186.83),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(120)))),
        new AprilTag(
                10,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(481.39),
                                Units.inchesToMeters(158.50),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(180)))),
        new AprilTag(
                11,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(497.77),
                                Units.inchesToMeters(130.17),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(240)))),
        new AprilTag(
                12,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(33.51),
                                Units.inchesToMeters(25.80),
                                Units.inchesToMeters(58.50)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(54)))),
        new AprilTag(
                13,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(33.51),
                                Units.inchesToMeters(291.20),
                                Units.inchesToMeters(58.50)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(306)))),
        new AprilTag(
                14,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(325.68),
                                Units.inchesToMeters(241.64),
                                Units.inchesToMeters(73.54)),
                        new Rotation3d(
                                Units.degreesToRadians(30),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(180)))),
        new AprilTag(
                15,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(325.68),
                                Units.inchesToMeters(75.39),
                                Units.inchesToMeters(73.54)),
                        new Rotation3d(
                                Units.degreesToRadians(30),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(180)))),
        new AprilTag(
                16,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(235.73),
                                Units.inchesToMeters(-0.15),
                                Units.inchesToMeters(51.25)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(90)))),
        new AprilTag(
                17,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(160.39),
                                Units.inchesToMeters(130.17),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(240)))),
        new AprilTag(
                18,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(144.0),
                                Units.inchesToMeters(158.50),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(180)))),
        new AprilTag(
                19,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(160.39),
                                Units.inchesToMeters(186.83),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(120)))),
        new AprilTag(
                20,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(193.10),
                                Units.inchesToMeters(186.83),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(60)))),
        new AprilTag(
                21,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(209.49),
                                Units.inchesToMeters(158.50),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0)))),
        new AprilTag(
                22,
                new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(193.10),
                                Units.inchesToMeters(130.17),
                                Units.inchesToMeters(12.13)),
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(300)))),
    };

    public static AprilTag getAprilTag(int id) {
        return new AprilTag(id, VisionConstants.APRIL_TAG_LAYOUT.getTagPose(id).get());
    }

    /*
     * To properly use the auto flip feature, the poses MUST be for the blue alliance.
     * The auto flip feature will automatically flip the poses for the red alliance.
     */
    public static final Pose2d BLUE_STARTING_POSE =
            new Pose2d(new Translation2d(0, 0), new Rotation2d());

    public static Translation2d getClosestPointOnLine(
            Translation2d startLine, Translation2d endLine) {
        Translation2d robotPoint = RobotContainer.s_Swerve.getPose().getTranslation();

        Vector2 lineVector =
                new Vector2(endLine.getX() - startLine.getX(), endLine.getY() - startLine.getY());
        Vector2 pointVector =
                new Vector2(
                        robotPoint.getX() - startLine.getX(), robotPoint.getY() - startLine.getY());

        double lineLengthSquared = lineVector.dot(lineVector);
        double dotProduct = pointVector.dot(pointVector);

        double t = dotProduct / lineLengthSquared; // projection of point onto line
        double lineLength = startLine.getDistance(endLine);
        double percentToIgnoreFromEachSide =
                (RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0) / lineLength;

        t = Math.max(percentToIgnoreFromEachSide, Math.min(1 - percentToIgnoreFromEachSide, t));

        return new Translation2d(
                startLine.getX() + t * lineVector.x, startLine.getY() + t * lineVector.y);
    }
}
