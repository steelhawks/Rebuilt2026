package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.Constants.RobotConstants;

import static edu.wpi.first.units.Units.Meters;

public class ShooterStructure {

    private static final InterpolatingDoubleTreeMap shootingTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shootingFlywheelVelocityMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingTreeMap<Double, Rotation2d> shootingHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    private static final InterpolatingDoubleTreeMap ferryTimeOfFlightMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ferryFlywheelVelocityMap =
        new InterpolatingDoubleTreeMap();
    private static final InterpolatingTreeMap<Double, Rotation2d> ferryHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

    private static final double minShootDistance;
    private static final double maxShootDistance;
    private static final double minFerryDistance;
    private static final double maxFerryDistance;

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
        SubsystemConstants.LUTConstants c =
            switch (Constants.getRobot()) {
                case ALPHABOT -> SubsystemConstants.AlphaBot.LUT;
                case OMEGABOT, SIMBOT -> SubsystemConstants.OmegaBot.LUT;
                default -> SubsystemConstants.LUTConstants.UNSET;
            };
//        minShootDistance = c.minShootDistance();
//        maxShootDistance = c.maxShootDistance();

        minShootDistance = 1.146;
        maxShootDistance = 4.057;

        minFerryDistance = c.minFerryDistance();
        maxFerryDistance = c.maxFerryDistance();
         if (c.shootingTimeOfFlightMap() != null) {
             for (double[] entry : c.shootingTimeOfFlightMap()) {
                 shootingTimeOfFlightMap.put(entry[0], entry[1]);
             }
         }
//         if (c.shootingFlywheelVelocityMap() != null) {
//             for (double[] entry : c.shootingFlywheelVelocityMap()) {
//                 shootingFlywheelVelocityMap.put(entry[0], entry[1]);
//             }
//         }

        // 5.512 is bad, just a big jump so its kinda poorly tuned

        shootingFlywheelVelocityMap.put(1.146, 10.0);
        shootingFlywheelVelocityMap.put(1.633, 10.5);
        shootingFlywheelVelocityMap.put(2.639, 12.2);
        shootingFlywheelVelocityMap.put(3.3315, 13.1);
        shootingFlywheelVelocityMap.put(3.965, 13.5);
        shootingFlywheelVelocityMap.put(4.057, 15.0);
        shootingFlywheelVelocityMap.put(5.512, 13.0); // 5.512 is pretty inconsistnet
//         if (c.shootingHoodAngleMap() != null) {
//             for (double[] entry : c.shootingHoodAngleMap()) {
//                 shootingHoodAngleMap.put(entry[0], Rotation2d.fromRadians(entry[1]));
//             }
//         }
        shootingHoodAngleMap.put(1.146, Rotation2d.fromDegrees(80.0));
        shootingHoodAngleMap.put(1.633, Rotation2d.fromDegrees(78.0));
        shootingHoodAngleMap.put(2.639, Rotation2d.fromDegrees(76.0));
        shootingHoodAngleMap.put(3.3315, Rotation2d.fromDegrees(74.0));
        shootingHoodAngleMap.put(3.965, Rotation2d.fromDegrees(72.0));
        shootingHoodAngleMap.put(4.057, Rotation2d.fromDegrees(71.0));
        shootingHoodAngleMap.put(5.512, Rotation2d.fromDegrees(40.0));

         if (c.ferryTimeOfFlightMap() != null) {
             for (double[] entry : c.ferryTimeOfFlightMap()) {
                 ferryTimeOfFlightMap.put(entry[0], entry[1]);
             }
         }
         if (c.ferryFlywheelVelocityMap() != null) {
             for (double[] entry : c.ferryFlywheelVelocityMap()) {
                 ferryFlywheelVelocityMap.put(entry[0], entry[1]);
             }
         }
         if (c.ferryHoodAngleMap() != null) {
             for (double[] entry : c.ferryHoodAngleMap()) {
                 ferryHoodAngleMap.put(entry[0], Rotation2d.fromDegrees(entry[1]));
             }
         }
    }

    public static boolean isNoSolution(ProjectileData data) {
        return data == null || Double.isNaN(data.exitVelocity()) || Double.isNaN(data.hoodAngle());
    }

    public static double calculateTimeOfFlight(double v, double theta, double x, double deltaH) {
        // rearranged 0.5*g*t^2 - v*sin(theta)*t + deltaH = 0
//        if (Toggles.useLUT.get() && !Toggles.useKinematicsTOF.get()) {
//            return shootingTimeOfFlightMap.get(MathUtil.clamp(x, minShootDistance, maxShootDistance));
//        }
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
        var turretTranslation = new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(RobotConstants.ROBOT_TO_TURRET)
            .toPose2d()
            .getTranslation();
        return turretTranslation.getDistance(target.toTranslation2d());
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
            if (isFixedPitch) {
                return calculateShotFixedPitch(actualTarget, predictedTarget);
            }
            double x_dist = MathUtil.clamp(distanceToTarget(predictedTarget), minShootDistance, maxShootDistance);
            if (Toggles.useLUT.getAsBoolean()) {
                return new ProjectileData(
                    shootingFlywheelVelocityMap.get(x_dist),
                    shootingHoodAngleMap.get(x_dist).getRadians(),
                    predictedTarget);
            }
            double y_dist = predictedTarget
                .getMeasureZ()
                .minus(RobotConstants.ROBOT_TO_TURRET.getMeasureZ()).in(Meters);
            double r = FieldConstants.Hub.FUNNEL_RADIUS
                * x_dist
                / distanceToTarget(actualTarget);
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
         * @param actualTarget   The real hub target.
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
            double y = -turretHeightAboveField(); // ferry shot is on the ground so simplfiies to this from 0 - turret_height

            double theta = Math.PI / 4.0;
            double cos = Math.cos(theta);
            double v0 = Math.sqrt((G * x * x / 2) * (cos * cos * (x * Math.tan(theta) - y)));

            return new ProjectileData(v0, theta, new Translation3d(actualTarget.getX(), actualTarget.getY(), 0.0));
        }
    }

    public static class Moving {
        
        public static MovingShotSolution solveMovingShot(
            Translation3d actualTarget,
            Translation3d robotVelocity,
            Rotation2d robotHeading,
            int maxIterations,
            double timeTolerance
        ) {
            Translation2d fieldRelativeVelocity =
                new Translation2d(robotVelocity.getX(), robotVelocity.getY()).rotateBy(robotHeading);
            Translation3d fieldVelocity =
                new Translation3d(fieldRelativeVelocity.getX(), fieldRelativeVelocity.getY(), 0.0);
            double deltaH = actualTarget.getZ() - turretHeightAboveField();
            Translation3d virtualTarget = actualTarget;
            double virtualDist = MathUtil.clamp(distanceToTarget(actualTarget), minShootDistance, maxShootDistance);
            var projectile = Static.calculateShot(actualTarget, actualTarget, false);
            double v = projectile.exitVelocity();
            double theta = projectile.hoodAngle();
            double tGuess = calculateTimeOfFlight(v, theta, virtualDist, deltaH);

            for (int i = 0; i < maxIterations; i++) {
                virtualTarget = new Translation3d(
                    actualTarget.getX() - fieldVelocity.getX() * tGuess,
                    actualTarget.getY() - fieldVelocity.getY() * tGuess,
                    actualTarget.getZ());
                virtualDist = MathUtil.clamp(distanceToTarget(virtualTarget), minShootDistance, maxShootDistance);

                projectile = Static.calculateShot(virtualTarget, virtualTarget, false);
                v = projectile.exitVelocity();
                theta = projectile.hoodAngle();
                double newTof = calculateTimeOfFlight(v, theta, virtualDist, deltaH);

                Logger.recordOutput("SOTM/Iteration", i);
                Logger.recordOutput("SOTM/VirtualTarget", virtualTarget);
                Logger.recordOutput("SOTM/VirtualDistance", virtualDist);
                Logger.recordOutput("SOTM/TOF", newTof);
                Logger.recordOutput("SOTM/ExitVelocity", v);
                Logger.recordOutput("SOTM/HoodAngleDeg", Math.toDegrees(theta));

                if (Math.abs(newTof - tGuess) < timeTolerance) {
                    Logger.recordOutput("SOTM/ConvergedIterations", i + 1);
                    break;
                }
                tGuess = newTof;
            }
            var turretTranslation = new Pose3d(RobotState.getInstance().getEstimatedPose())
                .transformBy(RobotConstants.ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation();
            double fieldRelativeAngle = Math.atan2(
                virtualTarget.getY() - turretTranslation.getY(),
                virtualTarget.getX() - turretTranslation.getX());
            double turretMountYaw = RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
            double turretRelativeAngle = MathUtil.angleModulus(
                fieldRelativeAngle - robotHeading.getRadians() - turretMountYaw);
            Logger.recordOutput("SOTM/TurretRelativeAngleDeg", Math.toDegrees(turretRelativeAngle));
            return new MovingShotSolution(v, theta, Rotation2d.fromRadians(turretRelativeAngle), virtualTarget, tGuess);
        }
    }
}
