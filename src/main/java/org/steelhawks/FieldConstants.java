

package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.dyn4j.geometry.Vector2;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AprilTag;

public class FieldConstants {

    public static Translation2d getClosestPointOnLine(
        Translation2d startLine, Translation2d endLine) {
        Translation2d robotPoint = RobotState.getInstance().getEstimatedPose().getTranslation();

        Vector2 lineVector =
            new Vector2(endLine.getX() - startLine.getX(), endLine.getY() - startLine.getY());
        Vector2 pointVector =
            new Vector2(
                robotPoint.getX() - startLine.getX(), robotPoint.getY() - startLine.getY());

        double lineLengthSquared = lineVector.dot(lineVector);
        double dotProduct = pointVector.dot(lineVector);

        double t = dotProduct / lineLengthSquared; // projection of point onto line
        double lineLength = startLine.getDistance(endLine);
        double percentToIgnoreFromEachSide =
            (RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0) / lineLength;

        t = Math.max(percentToIgnoreFromEachSide, Math.min(1 - percentToIgnoreFromEachSide, t));

        return new Translation2d(
            startLine.getX() + t * lineVector.x, startLine.getY() + t * lineVector.y);
    }

    public static AprilTag getAprilTag(int id) {
        return new AprilTag(id, VisionConstants.APRIL_TAG_LAYOUT.getTagPose(id).get());
    }

    public static final double FIELD_LENGTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldLength();
    public static final double FIELD_WIDTH = VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();

    public static final Field2d FIELD_2D = new Field2d();

    public final static class Hub {
        public static final Translation2d HUB_CENTER =
            new Translation2d(Units.inchesToMeters(158.6 + (47.0 / 2.0)), FIELD_WIDTH / 2.0);
        public static final Translation3d HUB_CENTER_3D =
            new Translation3d(HUB_CENTER.getX(), HUB_CENTER.getY(), Units.inchesToMeters(72.0));

        public static final double FUNNEL_RADIUS = Units.inchesToMeters(24.0);
        public static final double FUNNEL_HEIGHT = Units.inchesToMeters(72.0);
        public static final double DISTANCE_ABOVE_FUNNEL_TO_CLEAR = Units.inchesToMeters(7.0);
    }

    public final static class Ferrying {
        public static final double FERRY_LINE_X = Units.inchesToMeters(45.0 + 5.0);
        public static final double ST_FERRY_LINE_Y = Units.inchesToMeters(65.65);
        public static final double EN_FERRY_LINE_Y = FIELD_WIDTH - Units.inchesToMeters(65.65);

        public static final Translation2d START_LINE = new Translation2d(FERRY_LINE_X, ST_FERRY_LINE_Y);
        public static final Translation2d END_LINE = new Translation2d(FERRY_LINE_X, EN_FERRY_LINE_Y);
    }
}
