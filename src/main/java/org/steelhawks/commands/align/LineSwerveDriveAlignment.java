package org.steelhawks.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Vector2;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.util.Maths;

import java.util.function.DoubleSupplier;

/**
 * Aligns to the closest point on a line. Afterwards, it uses the xSupplier and the ySupplier to allow the driver to translate across a fixed line.
 *
 * @author Farhan Jamil
 *
 * @see SwerveDriveAlignment
 */
public class LineSwerveDriveAlignment extends SwerveDriveAlignment {

    private static final double DEADBAND = 0.1;

    private final Translation2d lineStart;
    private final Translation2d lineEnd;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    /**
     * @param desiredRotation Desired rotational heading.
     * @param lineStart The start of the line to lock to.
     * @param lineEnd The end of the line to lock to.
     * @param xSupplier The controller's x-axis for manipulating translation.
     * @param ySupplier The controller's y-axis for manipulating translation.
     */
    public LineSwerveDriveAlignment(
        Rotation2d desiredRotation, Translation2d lineStart, Translation2d lineEnd, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(new Pose2d(FieldConstants.getClosestPointOnLine(lineStart, lineEnd), desiredRotation));

        this.lineStart = lineStart;
        this.lineEnd = lineEnd;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    private void setSetpoint(Pose2d setpoint) {
        targetPose = () -> setpoint;
    }

    /**
     * @return t in [0,1] so that
     * pointOnSegment = start + (end-start) * t,
     * clamped to ignore bumper over-travel
     */
    private double getClampedTOnSegment(
        Translation2d start, Translation2d end, Translation2d point) {
        Translation2d lineVec = end.minus(start); // line vector
        Translation2d ptVec = point.minus(start); // from start to point

        double lineLenSq = Math.pow(lineVec.getNorm(), 2); // line squared
        double rawDot = Maths.toVector2(ptVec).dot(Maths.toVector2(lineVec)); // projection numerator
        double rawT = rawDot / lineLenSq; // how far along [0..1] un-clamped

        // clamp so robot doesn’t drive off the ends
        double bumper = RobotConstants.ROBOT_LENGTH_WITH_BUMPERS / 2.0;
        double percentIgnore = bumper / Math.sqrt(lineLenSq);
        return edu.wpi.first.math.MathUtil.clamp(rawT, percentIgnore, 1 - percentIgnore);
    }

    @Override
    public void execute() {
        double x = -edu.wpi.first.math.MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
        double y = -edu.wpi.first.math.MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);

        // vector from start-end
        Vector2 lineVec = Maths.toVector2(lineEnd).subtract(Maths.toVector2(lineStart));
        Vector2 lineDir = lineVec.divide(lineVec.getNormalized().getMagnitude()); // normalize

        Vector2 raw = new Vector2(x, y); // raw input after deadband
        double distAlongLine = raw.dot(lineDir); // dot product gives "how much of raw is along lineDir"

        // get current aligned point on the line
        Pose2d current = targetPose.get();
        // clamp to stay between endpoints (t in [0,1])
        double t = getClampedTOnSegment(lineStart, lineEnd, current.getTranslation());
        double newT = edu.wpi.first.math.MathUtil.clamp(t + distAlongLine / lineVec.getNormalized().getMagnitude(), 0, 1);
        Vector2 newPos = Maths.toVector2(lineStart).add(lineVec.multiply(newT));

        setSetpoint(new Pose2d(Maths.toTranslation2d(newPos), current.getRotation()));
        super.execute();
    }
}
