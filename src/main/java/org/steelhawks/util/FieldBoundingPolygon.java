package org.steelhawks.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

import java.util.function.Supplier;
/**
 * A trigger that checks if the robot is within a specified bounding polygon on the field.
 * The bounding polygon is defined by its vertices.
 * <p>
 * <a href="https://en.wikipedia.org/wiki/Point_in_polygon">Point in Polygon Algorithm</a>
 * @author Farhan Jamil
 */
public class FieldBoundingPolygon extends Trigger {

    private final Vector2[] points;
    private final String name;

    public FieldBoundingPolygon(
        String name,
        Supplier<Pose2d> robotPose,
        Vector2... points) {
        super(() ->
            contains(
                new Vector2(
                    robotPose.get().getTranslation().getX(),
                    robotPose.get().getTranslation().getY()), points));
        this.points = points;
        this.name = name;

        if (Constants.getRobot() != Constants.RobotType.SIMBOT)
            return;
        new VirtualSubsystem("FieldBoundingPolygon/" + name) {
            @Override
            public void periodic() {
                Vector2[] dynamicPoints = new Vector2[points.length];
                for (int i = 0; i < points.length; i++) {
                    dynamicPoints[i] = new Vector2(
                        AllianceFlip.applyX(points[i].x),
                        AllianceFlip.applyY(points[i].y));
                }
                Logger.recordOutput("FieldBoundingPolygon/" + name + "/Points", Conversions.toTranslation2dArray(dynamicPoints));
            }
        };
    }

    public static boolean contains(Vector2 testPoint, Vector2[] points) {
        int numPoints = points.length;
        boolean result = false;
        for (int i = 0, j = numPoints - 1; i < numPoints; j = i++) {
            Vector2 p1 = Conversions.toVector2(AllianceFlip.apply(Conversions.toTranslation2d(points[i])));
            Vector2 p2 = Conversions.toVector2(AllianceFlip.apply(Conversions.toTranslation2d(points[j])));

            boolean intersects = ((p1.y <= testPoint.y && testPoint.y < p2.y) ||
                (p2.y <= testPoint.y && testPoint.y < p1.y)) &&
                (testPoint.x < (p2.x - p1.x) * (testPoint.y - p1.y) / (p2.y - p1.y + 1e-10) + p1.x);
            if (intersects) result = !result;
        }
        return result;
    }

}
