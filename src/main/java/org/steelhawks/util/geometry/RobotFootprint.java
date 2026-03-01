package org.steelhawks.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Represents the dynamic footprint of the robot on the field.
 *
 * <p>The base footprint is a rectangle defined by bumper half-widths.
 * Extensions (e.g. an intake) are attached at a given robot-relative angle
 * and report their current extension length via a {@link DoubleSupplier}.
 *
 * <p>All measurements are in meters.
 */
public class RobotFootprint {

    /** A named mechanism that extends from the robot center in a given robot-relative direction. */
    public static class Extension {
        private final String name;
        /** Robot-relative angle this extension protrudes toward. */
        private final Rotation2d direction;
        /** Current extension length in meters. */
        private final DoubleSupplier extensionMeters;

        /**
         * @param name           human-readable label (e.g. "intake")
         * @param direction      robot-relative direction of protrusion
         * @param extensionMeters current length supplier (meters from robot perimeter)
         */
        public Extension(String name, Rotation2d direction, DoubleSupplier extensionMeters) {
            this.name = name;
            this.direction = direction;
            this.extensionMeters = extensionMeters;
        }

        public String getName() { return name; }
        public Rotation2d getDirection() { return direction; }
        public double getExtensionMeters() { return extensionMeters.getAsDouble(); }
    }

    private final double bumperHalfX; // half-width along robot X axis (front/back), meters
    private final double bumperHalfY; // half-width along robot Y axis (left/right), meters
    private final List<Extension> extensions = new ArrayList<>();

    /**
     * @param bumperWidthX full bumper width front-to-back in meters
     * @param bumperWidthY full bumper width left-to-right in meters
     */
    public RobotFootprint(double bumperWidthX, double bumperWidthY) {
        this.bumperHalfX = bumperWidthX / 2.0;
        this.bumperHalfY = bumperWidthY / 2.0;
    }

    /** Add a dynamic extension to this footprint (e.g. intake, climber). */
    public RobotFootprint withExtension(Extension extension) {
        extensions.add(extension);
        return this;
    }

    /**
     * Returns all field-frame points that define the robot's current footprint
     * given the robot's current {@link Pose2d}.
     *
     * <p>Includes the 4 bumper corners plus the tip of every extension.
     */
    public List<Translation2d> getPoints(Pose2d robotPose) {
        List<Translation2d> points = new ArrayList<>();
        // 4 bumper corners in robot-relative coords, rotated into field frame
        double[][] corners = {
            { bumperHalfX,  bumperHalfY},
            { bumperHalfX, -bumperHalfY},
            {-bumperHalfX, -bumperHalfY},
            {-bumperHalfX,  bumperHalfY}
        };
        for (double[] corner : corners) {
            points.add(toFieldFrame(robotPose, corner[0], corner[1]));
        }
        // Extension tips — measured from robot center, so we add the bumper
        // projection in the extension direction plus the mechanism extension
        for (Extension ext : extensions) {
            double extLength = ext.getExtensionMeters();
            if (extLength <= 0.0) continue;
            // Field-frame angle = robot heading + robot-relative extension angle
            Rotation2d fieldAngle = robotPose.getRotation().plus(ext.getDirection());
            // Bumper surface offset: project bumper rectangle onto the extension direction
            double bumpOffset = bumperProjection(ext.getDirection());

            double totalReach = bumpOffset + extLength;
            double dx = totalReach * fieldAngle.getCos();
            double dy = totalReach * fieldAngle.getSin();

            points.add(robotPose.getTranslation().plus(new Translation2d(dx, dy)));
        }
        return points;
    }

    /**
     * Returns the distance from the robot center to the bumper surface
     * in the given robot-relative direction (rectangle ray-cast).
     */
    private double bumperProjection(Rotation2d robotRelativeAngle) {
        double cos = Math.abs(robotRelativeAngle.getCos());
        double sin = Math.abs(robotRelativeAngle.getSin());
        // Ray-AABB intersection for a box centered at origin
        double tx = (cos > 1e-6) ? bumperHalfX / cos : Double.MAX_VALUE;
        double ty = (sin > 1e-6) ? bumperHalfY / sin : Double.MAX_VALUE;
        return Math.min(tx, ty);
    }

    private Translation2d toFieldFrame(Pose2d pose, double robotX, double robotY) {
        Rotation2d heading = pose.getRotation();
        double fx = pose.getX() + robotX * heading.getCos() - robotY * heading.getSin();
        double fy = pose.getY() + robotX * heading.getSin() + robotY * heading.getCos();
        return new Translation2d(fx, fy);
    }
}
