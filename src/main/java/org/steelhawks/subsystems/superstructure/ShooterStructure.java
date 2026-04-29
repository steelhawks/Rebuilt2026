package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.steelhawks.*;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.util.AllianceFlip;

import static edu.wpi.first.units.Units.Meters;

public class ShooterStructure {

    private static final InterpolatingDoubleTreeMap shootingTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shootingFlywheelVelocityMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingTreeMap<Double, Rotation2d> shootingHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    private static final LoggedNetworkNumber lutDistanceOffsetMeters =
        new LoggedNetworkNumber("ShooterStructure/LUTDistanceOffsetMeters", Units.feetToMeters(0.0)); // 1.95ft is what we had on the field

    private static double minShootDistance;
    private static double maxShootDistance;

    public record ProjectileData(double exitVelocity, double hoodAngle, Translation3d target) {}
    public record MovingShotSolution(
        double exitVelocity,
        double hoodAngleRad,
        Rotation2d turretAngle,
        Translation3d virtualTarget,
        double timeOfFlight
    ) {}
    public static final ProjectileData kNoSolution = new ProjectileData(Double.NaN, Double.NaN, new Translation3d());
    private static final double G = 9.81;

    static {
        loadLUTSoft();
    }

    public static boolean isNoSolution(ProjectileData data) {
        return data == null || Double.isNaN(data.exitVelocity()) || Double.isNaN(data.hoodAngle());
    }

    public static double calculateTimeOfFlight(double v, double theta, double x, double deltaH) {
        // rearranged 0.5*g*t^2 - v*sin(theta)*t + deltaH = 0
        if (Toggles.useLUT.get() && !Toggles.useKinematicsTOF.get()) {
            return shootingTimeOfFlightMap.get(MathUtil.clamp(x, minShootDistance, maxShootDistance));
        }
        double a = 0.5 * G;
        double b = -v * Math.sin(theta);
        double c = deltaH;
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return Double.NaN;
        double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);
        return (t1 > 0) ? t1 : t2;
    }

    public static double angularToLinearVelocity(double angularVelocityRadPerSec, double radius) {
        return angularVelocityRadPerSec * radius;
    }

    public static double linearToAngularVelocity(double linearVelocityMetersPerSec, double radius) {
        return (linearVelocityMetersPerSec) / radius;
    }

    public static double distanceToTarget(Translation3d target) {
        return distanceToTarget(target, getTurretTranslation());
    }

    private static double distanceToTarget(Translation3d target, Translation2d turretXY) {
        return turretXY.getDistance(target.toTranslation2d());
    }

    /** Computes the turret's field-frame XY position from the current estimated pose. */
    static Translation2d getTurretTranslation() {
        return new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(RobotConstants.ROBOT_TO_TURRET)
            .toPose2d()
            .getTranslation();
    }

    /**
     * Returns the turret height above the field floor in meters.
     */
    private static double turretHeightAboveField() {
        return RobotConstants.ROBOT_TO_TURRET.getZ();
    }

    public static class Static {

        public static ProjectileData calculateShot(Translation3d target, Translation3d predictedTarget) {
            return calculateShot(target, predictedTarget, false);
        }

        public static ProjectileData calculateShot(
            Translation3d actualTarget, Translation3d predictedTarget, boolean isFixedPitch
        ) {
            return calculateShot(actualTarget, predictedTarget, isFixedPitch, -1.0);
        }

        /**
         * Same as {@link #calculateShot} but accepts a pre-computed distance to avoid
         * recomputing the Pose3d turret transform chain. Pass -1 to compute it fresh.
         */
        public static ProjectileData calculateShot(
            Translation3d actualTarget, Translation3d predictedTarget, boolean isFixedPitch, double precomputedDist
        ) {
            if (isFixedPitch) {
                return calculateShotFixedPitch(actualTarget, predictedTarget);
            }
            double rawDist = precomputedDist >= 0 ? precomputedDist : distanceToTarget(predictedTarget);
            double x_dist = MathUtil.clamp(rawDist, minShootDistance, maxShootDistance);
            if (Toggles.useLUT.getAsBoolean()) {
                double lutDist = MathUtil.clamp(rawDist + lutDistanceOffsetMeters.get(), minShootDistance, maxShootDistance);
                return new ProjectileData(
                    shootingFlywheelVelocityMap.get(lutDist),
                    shootingHoodAngleMap.get(lutDist).getRadians(),
                    predictedTarget);
            }
            double y_dist = predictedTarget
                .getMeasureZ()
                .minus(RobotConstants.ROBOT_TO_TURRET.getMeasureZ()).in(Meters);
            // When actualTarget == predictedTarget (the SOTM loop case), r simplifies to FUNNEL_RADIUS
            // Use precomputedDist to avoid a second distanceToTarget() call
            double actualDist = precomputedDist >= 0 ? precomputedDist : distanceToTarget(actualTarget);
            double r = FieldConstants.Hub.FUNNEL_RADIUS
                * x_dist
                / actualDist;
            double h = FieldConstants.Hub.FUNNEL_HEIGHT + FieldConstants.Hub.DISTANCE_ABOVE_FUNNEL_TO_CLEAR;
            double A1 = x_dist * x_dist;
            double B1 = x_dist;
            double D1 = y_dist;
            double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
            double B2 = -r;
            double D2 = h;
            double Bm = -B2 / B1;
            double A3 = Bm * A1 + A2;
            double D3 = Bm * D1 + D2;
            double a = D3 / A3;
            double b = (D1 - A1 * a) / B1;
            double theta = Math.atan(b);
            double v0 = Math.sqrt(-G / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
            if (Double.isNaN(v0) || Double.isNaN(theta)) {
                v0 = 0;
                theta = 0;
            }
            return new ProjectileData(
                v0,
                theta,
                predictedTarget);
        }

        /**
         * Computes a ballistic launch solution for a shooter with a fixed pitch angle.
         *
         * @param actualTarget    The real hub target.
         * @param predictedTarget Used for shooting on the move.
         * @return ProjectileData, or kNoSolution if the fixed angle cannot clear the funnel or reach the target.
         */
        private static ProjectileData calculateShotFixedPitch(
            Translation3d actualTarget, Translation3d predictedTarget
        ) {
            final double theta = RobotConstants.FIXED_SHOOTER_ANGLE;
            double cosTheta = Math.cos(theta);
            double tanTheta = Math.tan(theta);
            double turretH = turretHeightAboveField();
            double x = distanceToTarget(predictedTarget);
            double y = predictedTarget.getZ() - turretH;
            double denom = 2 * cosTheta * cosTheta * (x * tanTheta - y);
            if (denom <= 0) {
                return kNoSolution;
            }
            double v0 = Toggles.useLUT.get() ? shootingFlywheelVelocityMap.get(x) : Math.sqrt(G * x * x / denom);
            if (Double.isNaN(v0) || Double.isInfinite(v0)) {
                return kNoSolution;
            }
            double r = FieldConstants.Hub.FUNNEL_RADIUS * x / distanceToTarget(actualTarget);
            double xFunnel = x - r;
            double requiredHeight = (FieldConstants.Hub.FUNNEL_HEIGHT
                + FieldConstants.Hub.DISTANCE_ABOVE_FUNNEL_TO_CLEAR)
                - turretH;
            double yAtFunnel = xFunnel * tanTheta
                - (G * xFunnel * xFunnel) / (2.0 * v0 * v0 * cosTheta * cosTheta);
            if (yAtFunnel < requiredHeight) {
                return kNoSolution;
            }
            return new ProjectileData(v0, theta, predictedTarget);
        }

        public static ProjectileData calculateFerryShot(Translation2d actualTarget) {
            double x = distanceToTarget(new Translation3d(actualTarget.getX(), actualTarget.getY(), 0.0));
            double y = -turretHeightAboveField(); // ferry shot is on the ground so simplifies to this from 0 - turret_height

            double theta = Math.PI / 4.0;
            double cos = Math.cos(theta);
            double denom = 2.0 * cos * cos * (x * Math.tan(theta) - y);
            double v0 = Math.sqrt(G * x * x / denom);

            return new ProjectileData(v0, theta, new Translation3d(actualTarget.getX(), actualTarget.getY(), 0.0));
        }

        public static Translation2d calculateFerryShotSetpoint() {
            double robotYBlue = AllianceFlip.shouldFlip()
                ? FieldConstants.FIELD_WIDTH - RobotState.getInstance().getEstimatedPose().getY()
                : RobotState.getInstance().getEstimatedPose().getY();

            double offset = FieldConstants.Ferrying.DISTANCE_OFFSET;
            Translation2d segEnd = (robotYBlue < FieldConstants.FIELD_WIDTH / 2.0)
                ? FieldConstants.Ferrying.MID_LINE.minus(new Translation2d(0.0, offset))
                : FieldConstants.Ferrying.MID_LINE.plus(new Translation2d(0.0, offset));
            Translation2d segStart = (robotYBlue < FieldConstants.FIELD_WIDTH / 2.0)
                ? FieldConstants.Ferrying.START_LINE
                : FieldConstants.Ferrying.END_LINE;

            return AllianceFlip.apply(FieldConstants.getClosestPointOnLine(segStart, segEnd));
        }
    }

    public static class Moving {
        
        public static MovingShotSolution solveMovingShot(
            Translation3d actualTarget,
            Translation3d robotVelocity,
            Rotation2d robotHeading,
            double chassisOmegaRadPerSec,
            int maxIterations,
            double timeTolerance
        ) {
            boolean isFerry = RobotState.getInstance().getAimState().equals(AimState.FERRY);
            Translation2d turretXY = getTurretTranslation();
            // add turrets tangential velocity from chassis rotation.
            // for a point at (dx, dy) in robot frame rotating at omega rad/s:
            //v = (-omega*dy, omega*dx) cross product
            double turretDx = RobotConstants.ROBOT_TO_TURRET.getX();
            double turretDy = RobotConstants.ROBOT_TO_TURRET.getY();
            Translation2d fieldRelativeVelocity =
                new Translation2d(
                    robotVelocity.getX() + (-chassisOmegaRadPerSec * turretDy),
                    robotVelocity.getY() + (chassisOmegaRadPerSec * turretDx)) // omega could be negative
                .rotateBy(robotHeading);

            double deltaH = actualTarget.getZ() - turretHeightAboveField();
            double velX = fieldRelativeVelocity.getX();
            double velY = fieldRelativeVelocity.getY();
            Translation3d virtualTarget = actualTarget;
            double rawDist = turretXY.getDistance(actualTarget.toTranslation2d());
            double virtualDist = MathUtil.clamp(rawDist, minShootDistance, maxShootDistance);
            var projectile =
                isFerry
                    ? Static.calculateFerryShot(actualTarget.toTranslation2d())
                    : Static.calculateShot(actualTarget, actualTarget, false, rawDist);
            double v = projectile.exitVelocity();
            double theta = projectile.hoodAngle();
            double tGuess = calculateTimeOfFlight(v, theta, virtualDist, deltaH);
            int convergedAt = maxIterations;
            for (int i = 0; i < maxIterations; i++) {
                virtualTarget = new Translation3d(
                    actualTarget.getX() - velX * tGuess,
                    actualTarget.getY() - velY * tGuess,
                    actualTarget.getZ());
                rawDist = turretXY.getDistance(virtualTarget.toTranslation2d());
                virtualDist = MathUtil.clamp(rawDist, minShootDistance, maxShootDistance);
                projectile = isFerry
                    ? Static.calculateFerryShot(virtualTarget.toTranslation2d())
                    : Static.calculateShot(virtualTarget, virtualTarget, false, rawDist);
                v = projectile.exitVelocity();
                theta = projectile.hoodAngle();
                double newTof = calculateTimeOfFlight(v, theta, virtualDist, deltaH);
                if (Math.abs(newTof - tGuess) < timeTolerance) {
                    convergedAt = i + 1;
                    tGuess = newTof;
                    break;
                }
                tGuess = newTof;
            }

            Logger.recordOutput("SOTM/VirtualTarget", virtualTarget);
            Logger.recordOutput("SOTM/VirtualDistance", virtualDist);
            Logger.recordOutput("SOTM/TOF", tGuess);
            Logger.recordOutput("SOTM/ExitVelocity", v);
            Logger.recordOutput("SOTM/HoodAngleDeg", Math.toDegrees(theta));
            Logger.recordOutput("SOTM/ConvergedIterations", convergedAt);

            double fieldRelativeAngle = Math.atan2(
                virtualTarget.getY() - turretXY.getY(),
                virtualTarget.getX() - turretXY.getX());
            double turretMountYaw = RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
            double turretRelativeAngle = MathUtil.angleModulus(
                fieldRelativeAngle - robotHeading.getRadians() - turretMountYaw);
            Logger.recordOutput("SOTM/TurretRelativeAngleDeg", Math.toDegrees(turretRelativeAngle));
            return new MovingShotSolution(v, theta, Rotation2d.fromRadians(turretRelativeAngle), virtualTarget, tGuess);
        }
    }

    private static void switchLUT(boolean softLUTOn) {
        SmartDashboard.putBoolean("ShooterStructure/UsingLUTSoft", softLUTOn);
    }

    public static void loadLUTHard() {
        minShootDistance = 1.146;
        maxShootDistance = 6.2;

        switchLUT(false);
        shootingFlywheelVelocityMap.clear();
        shootingHoodAngleMap.clear();
        shootingTimeOfFlightMap.clear();
//
//        shootingFlywheelVelocityMap.put(1.405, 12.3);
        shootingFlywheelVelocityMap.put(1.972, 11.0);
        shootingFlywheelVelocityMap.put(2.635, 11.2);
        shootingFlywheelVelocityMap.put(3.17, 11.5);
        shootingFlywheelVelocityMap.put(3.855, 12.0);
        shootingFlywheelVelocityMap.put(4.588, 12.7);
        shootingFlywheelVelocityMap.put(5.217, 13.2);
        shootingFlywheelVelocityMap.put(6.123, 13.8);
//
//        shootingHoodAngleMap.put(1.405, Rotation2d.fromDegrees(78));
        shootingHoodAngleMap.put(1.972, Rotation2d.fromDegrees(73));
        shootingHoodAngleMap.put(2.635, Rotation2d.fromDegrees(68));
        shootingHoodAngleMap.put(3.17, Rotation2d.fromDegrees(66));
        shootingHoodAngleMap.put(3.855, Rotation2d.fromDegrees(64));
        shootingHoodAngleMap.put(4.588, Rotation2d.fromDegrees(62));
        shootingHoodAngleMap.put(5.217, Rotation2d.fromDegrees(60));
        shootingHoodAngleMap.put(6.123, Rotation2d.fromDegrees(58));

//        shootingTimeOfFlightMap.put(1.405, 1.296);
        shootingTimeOfFlightMap.put(1.972, 1.23);
        shootingTimeOfFlightMap.put(2.635, 1.20);
        shootingTimeOfFlightMap.put(3.17, 1.21);
        shootingTimeOfFlightMap.put(3.855, 1.23);
        shootingTimeOfFlightMap.put(4.588, 1.31);
        shootingTimeOfFlightMap.put(5.217, 1.39);
        shootingTimeOfFlightMap.put(6.123, 1.38);
    }

    public static void loadLUTSoft() {
        minShootDistance = 1.401;
        maxShootDistance = 6.123;

        switchLUT(true);
        shootingFlywheelVelocityMap.clear();
        shootingHoodAngleMap.clear();
        shootingTimeOfFlightMap.clear();

        shootingFlywheelVelocityMap.put(1.401, 11.0);
        shootingFlywheelVelocityMap.put(1.965, 11.5);
        shootingFlywheelVelocityMap.put(2.651, 12.0);
        shootingFlywheelVelocityMap.put(3.078, 12.5);
        shootingFlywheelVelocityMap.put(3.823, 13.1);
        shootingFlywheelVelocityMap.put(4.616, 13.7);
        shootingFlywheelVelocityMap.put(5.221, 14.1);
        shootingFlywheelVelocityMap.put(6.123, 14.5);

        shootingHoodAngleMap.put(1.401, Rotation2d.fromDegrees(74));
        shootingHoodAngleMap.put(1.965, Rotation2d.fromDegrees(72));
        shootingHoodAngleMap.put(2.651, Rotation2d.fromDegrees(70));
        shootingHoodAngleMap.put(3.078, Rotation2d.fromDegrees(68));
        shootingHoodAngleMap.put(3.823, Rotation2d.fromDegrees(66));
        shootingHoodAngleMap.put(4.616, Rotation2d.fromDegrees(63));
        shootingHoodAngleMap.put(5.221, Rotation2d.fromDegrees(59));
        shootingHoodAngleMap.put(6.123, Rotation2d.fromDegrees(55));

        shootingTimeOfFlightMap.put(1.401, 1.277);
        shootingTimeOfFlightMap.put(1.965, 1.32);
        shootingTimeOfFlightMap.put(2.651, 1.433);
        shootingTimeOfFlightMap.put(3.078, 1.433);
        shootingTimeOfFlightMap.put(4.616, 1.493);
        shootingTimeOfFlightMap.put(5.221, 1.51);
        shootingTimeOfFlightMap.put(6.123, 1.493);
    }
}
