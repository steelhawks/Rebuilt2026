package org.steelhawks.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

/**
 * Represents a rectangular field boundary and provides WPILib {@link Trigger}s
 * that activate based on how the robot's footprint interacts with it.
 *
 * <p>Two activation modes are supported:
 * <ul>
 *   <li>{@link Mode#CENTER_ONLY} — triggers when the robot's odometry center enters the zone.</li>
 *   <li>{@link Mode#PERIMETER} — triggers when any point of the robot's dynamic footprint
 *       (bumpers + extended mechanisms) touches or enters the zone.</li>
 * </ul>
 */
public class Boundary {

    public enum Mode {
        /** Only the robot's pose center is checked against the boundary. */
        CENTER_ONLY,
        /** Every point of the robot footprint (bumpers + mechanism tips) is checked. */
        PERIMETER
    }

    private final Supplier<Rectangle2d> boundary;
    private final Supplier<Pose2d> poseSupplier;
    private final RobotFootprint footprint;

    /**
     * @param boundary the rectangular field zone
     * @param poseSupplier supplier of the robot's current field-frame {@link Pose2d}
     * @param footprint the robot's dynamic footprint geometry
     */
    public Boundary(Supplier<Rectangle2d> boundary, Supplier<Pose2d> poseSupplier, RobotFootprint footprint) {
        this.boundary = boundary;
        this.poseSupplier = poseSupplier;
        this.footprint = footprint;
    }

    /**
     * Returns a {@link BooleanSupplier} that is {@code true} when the robot is active
     * in this boundary according to the given {@link Mode}.
     */
    public BooleanSupplier isActive(Mode mode) {
        return switch (mode) {
            case CENTER_ONLY -> () -> boundary.get().contains(poseSupplier.get().getTranslation());
            case PERIMETER -> () -> footprint.getPoints(poseSupplier.get()).stream().anyMatch(boundary.get()::contains);
        };
    }

    /**
     * Returns a WPILib {@link Trigger} that activates when the robot is inside
     * the boundary according to the given {@link Mode}.
     */
    public Trigger asTrigger(Mode mode) {
        return new Trigger(isActive(mode));
    }

    /**
     * Convenience factory — creates a {@link Trigger} directly without holding
     * a {@code Boundary} reference.
     *
     * @param boundary the rectangular field zone
     * @param poseSupplier supplier of the robot's current field-frame {@link Pose2d}
     * @param footprint the robot's dynamic footprint geometry
     * @param mode which activation mode to use
     */
    public static Trigger asTrigger(
        Supplier<Rectangle2d> boundary,
        Supplier<Pose2d> poseSupplier,
        RobotFootprint footprint,
        Mode mode) {
        return new Boundary(boundary, poseSupplier, footprint).asTrigger(mode);
    }

    /**
     * Logs the boundary corners, current robot footprint probe points, and active
     * state for both modes to AdvantageKit. Call this once per {@code robotPeriodic()}.
     *
     * <p>AdvantageScope will render {@code Translation2d[]} as a path on the field view,
     * so the boundary draws as a closed box and the footprint points show as dots.
     *
     * @param key the NetworkTables key prefix (e.g. {@code "Boundaries/Stage"})
     */
    public void log(String key) {
        Pose2d robotPose = poseSupplier.get();

        Logger.recordOutput(key + "/boundary", getBoundaryCorners());

        List<Translation2d> footprintPoints = footprint.getPoints(robotPose);
        Logger.recordOutput(key + "/footprint", footprintPoints.toArray(new Translation2d[0]));

        Logger.recordOutput(key + "/centerActive", boundary.get().contains(robotPose.getTranslation()));
        Logger.recordOutput(key + "/perimeterActive", footprintPoints.stream().anyMatch(boundary.get()::contains));
    }

    private Translation2d[] getBoundaryCorners() {
        Translation2d center = boundary.get().getCenter().getTranslation();
        double halfX = boundary.get().getMeasureXWidth().in(Meters) / 2.0;
        double halfY = boundary.get().getMeasureYWidth().in(Meters) / 2.0;

        return new Translation2d[]{
            new Translation2d(center.getX() - halfX, center.getY() - halfY),
            new Translation2d(center.getX() + halfX, center.getY() - halfY),
            new Translation2d(center.getX() + halfX, center.getY() + halfY),
            new Translation2d(center.getX() - halfX, center.getY() + halfY),
            new Translation2d(center.getX() - halfX, center.getY() - halfY) // close the loop
        };
    }
}
