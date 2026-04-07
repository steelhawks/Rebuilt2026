

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

    public static final int HUB_HEIGHT = 48; //inches
    public static final int HUB_WIDTH = 41;
    public static final int HUB_FUNNEL_HEIGHT = 24;
    public static final int HUB_FUNNEL_WIDTH = 24;
    public static final int HUB_HEIGHT_TO_CLEAR_FUNNEL = 21;

    public static AprilTag getAprilTag(int id) {
        return new AprilTag(id, VisionConstants.APRIL_TAG_LAYOUT.getTagPose(id).get());
    }

    /*
     * To properly use the auto flip feature, the poses MUST be for the blue alliance.
     * The auto flip feature will automatically flip the poses for the red alliance.
     */
    public static final Pose2d BLUE_STARTING_POSE =
        new Pose2d(new Translation2d(0, 0), new Rotation2d());

    public static final Translation2d HUB_CENTER =
        new Translation2d(Units.inchesToMeters(158.6 + (47.0 / 2.0)), FIELD_WIDTH / 2.0);
    public static final Translation3d HUB_CENTER_3D =
        new Translation3d(HUB_CENTER.getX(), HUB_CENTER.getY(), 1.575);

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
