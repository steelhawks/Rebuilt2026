package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final ProjectileData kNoSolution = new ProjectileData(Double.NaN, Double.NaN, new Translation3d());
    private static final double G = 9.81;

    static {
        SubsystemConstants.LUTConstants c =
            switch (Constants.getRobot()) {
                case ALPHABOT -> SubsystemConstants.AlphaBot.LUT;
                case OMEGABOT, SIMBOT -> SubsystemConstants.OmegaBot.LUT;
                default -> SubsystemConstants.LUTConstants.UNSET;
            };
        minShootDistance = c.minShootDistance();
        maxShootDistance = c.maxShootDistance();

        minFerryDistance = c.minFerryDistance();
        maxFerryDistance = c.maxFerryDistance();
         if (c.shootingTimeOfFlightMap() != null) {
             for (double[] entry : c.shootingTimeOfFlightMap()) {
                 shootingTimeOfFlightMap.put(entry[0], entry[1]);
             }
         }
         if (c.shootingFlywheelVelocityMap() != null) {
             for (double[] entry : c.shootingFlywheelVelocityMap()) {
                 shootingFlywheelVelocityMap.put(entry[0], entry[1]);
             }
         }
         if (c.shootingHoodAngleMap() != null) {
             for (double[] entry : c.shootingHoodAngleMap()) {
                 shootingHoodAngleMap.put(entry[0], Rotation2d.fromRadians(entry[1]));
             }
         }
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
                 ferryHoodAngleMap.put(entry[0], Rotation2d.fromRadians(entry[1]));
             }
         }
    }

    public static boolean isNoSolution(ProjectileData data) {
        return data == null || Double.isNaN(data.exitVelocity()) || Double.isNaN(data.hoodAngle());
    }

    public static double calculateTimeofFlight(double exitVelocity, double pivotAngle, double distanceToTravel) {
        return distanceToTravel / (exitVelocity * Math.cos(pivotAngle));
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
            double x_dist = distanceToTarget(predictedTarget);
            if (Toggles.useLUT.getAsBoolean() && Constants.getRobot() == Constants.RobotType.OMEGABOT) {
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
    }

    public static class Moving {

        private static final int MAX_ITERATIONS = 5;
        private static final double CONVERGENCE_THRESHOLD = 0.01; // meters

        /**
         * Calculates shot parameters for shooting while moving (stationary target).
         *
         * @param actualTarget The actual target to score at.
         * @param isFixedPitch Whether this is a fixed pitch shooter.
         * @return ProjectileData or kNoSolution.
         */
        public static ProjectileData calculateMovingShot(
            Translation3d actualTarget,
            boolean isFixedPitch
        ) {
            return calculateMovingShot(actualTarget, new Translation3d(), isFixedPitch);
        }

        /**
         * Calculates shot parameters for shooting while moving (moving target).
         * <p>
         * Uses iterative time-of-flight prediction to find where to aim.
         *
         * @param actualTarget   The actual target to score at.
         * @param targetVelocity The velocity the target is moving at.
         * @param isFixedPitch   Whether this is a fixed pitch shooter.
         * @return ProjectileData or kNoSolution.
         */
        public static ProjectileData calculateMovingShot(
            Translation3d actualTarget,
            Translation3d targetVelocity,
            boolean isFixedPitch
        ) {
            Translation3d robotVelocity = new Translation3d(
                RobotContainer.s_Swerve.getChassisSpeeds().vxMetersPerSecond,
                RobotContainer.s_Swerve.getChassisSpeeds().vyMetersPerSecond,
                0.0);

            Translation3d predictedTarget = actualTarget;
            ProjectileData solution = kNoSolution;

            for (int i = 0; i < MAX_ITERATIONS; i++) {
                solution = Static.calculateShot(actualTarget, predictedTarget, isFixedPitch);

//                 Fixed: properly check kNoSolution instead of null
                if (isNoSolution(solution)) {
                    Logger.recordOutput("Shooter/Moving/ShotImpossible", true);
                    return kNoSolution;
                }

                double distance = distanceToTarget(predictedTarget);
                double tof = calculateTimeofFlight(solution.exitVelocity(), solution.hoodAngle(), distance);

                Translation3d relativeVelocity = targetVelocity.minus(robotVelocity);
                Translation3d newPredictedTarget = actualTarget.plus(relativeVelocity.times(tof));

                double error = predictedTarget.getDistance(newPredictedTarget);

                Logger.recordOutput("Shooter/Moving/IterationError_" + i, error);
                Logger.recordOutput("Shooter/Moving/HoodAngle_" + i, Math.toDegrees(solution.hoodAngle()));
                Logger.recordOutput("Shooter/Moving/ExitVelocity_" + i, solution.exitVelocity());

                if (error < CONVERGENCE_THRESHOLD) {
                    Logger.recordOutput("Shooter/Moving/ConvergedIterations", i + 1);
                    break;
                }

                predictedTarget = newPredictedTarget;
            }

            Logger.recordOutput("Shooter/Moving/FinalPredictedTarget",
                new double[]{predictedTarget.getX(), predictedTarget.getY(), predictedTarget.getZ()});
            Logger.recordOutput("Shooter/Moving/FinalHoodAngleDeg", Math.toDegrees(solution.hoodAngle()));
            Logger.recordOutput("Shooter/Moving/FinalExitVelocity", solution.exitVelocity());

            return solution;
        }
    }
}
