

package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AprilTag;

public class FieldConstants {

    public static Translation2d getClosestPointOnLine(
        Translation2d startLine, Translation2d endLine) {
        Translation2d robotPoint = RobotState.getInstance().getEstimatedPose().getTranslation();

        Translation2d lineVector =
            new Translation2d(endLine.getX() - startLine.getX(), endLine.getY() - startLine.getY());
        Translation2d pointVector =
            new Translation2d(
                robotPoint.getX() - startLine.getX(), robotPoint.getY() - startLine.getY());

        double lineLengthSquared = lineVector.dot(lineVector);
        double dotProduct = pointVector.dot(lineVector);
        double t = dotProduct / lineLengthSquared;
        double lineLength = Math.sqrt(lineLengthSquared);

        // Unit vector along the line
        double ux = lineVector.getX() / lineLength;
        double uy = lineVector.getY() / lineLength;
        // The robot half-dimension that matters is the one perpendicular to the line.
        // Project the robot's half-extents onto the perpendicular direction.
        // perpendicular = (-uy, ux)
        double halfX = RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0;
        double halfY = RobotConstants.ROBOT_WIDTH_WITH_BUMPERS / 2.0;
        // The margin along the line is the parallel projection of the robot's half-extents
        double parallelMargin = Math.abs(ux * halfX) + Math.abs(uy * halfY);
        double percentToIgnoreFromEachSide = parallelMargin / lineLength;
        t = Math.max(percentToIgnoreFromEachSide, Math.min(1 - percentToIgnoreFromEachSide, t));
        return new Translation2d(
            startLine.getX() + t * lineVector.getX(), startLine.getY() + t * lineVector.getY());
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
        public static final double FUNNEL_HEIGHT = Units.inchesToMeters(72.0 - 56.4);
        public static final double DISTANCE_ABOVE_FUNNEL_TO_CLEAR = Units.inchesToMeters(-2.0); // go back to 20 for max energy and if you want it to just fall straight into the chute, dont hit side walls
    }

    public final static class Ferrying {
        public static final double FERRY_LINE_X = Units.inchesToMeters(45.0 + 5.0);
        public static final double ST_FERRY_LINE_Y = Units.inchesToMeters(65.65);
        public static final double EN_FERRY_LINE_Y = FIELD_WIDTH - Units.inchesToMeters(65.65);

        public static final Translation2d START_LINE = new Translation2d(FERRY_LINE_X, ST_FERRY_LINE_Y);
        public static final Translation2d END_LINE = new Translation2d(FERRY_LINE_X, EN_FERRY_LINE_Y);
    }
}
