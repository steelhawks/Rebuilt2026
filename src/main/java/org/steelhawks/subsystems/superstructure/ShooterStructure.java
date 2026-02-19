package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.Toggles;

public class ShooterStructure {

    public record ProjectileData(double exitVelocity, double hoodAngle, Translation3d target) {}
    public static final ProjectileData kNoSolution = new ProjectileData(Double.NaN, Double.NaN, new Translation3d());
    private static final double G = 9.81;

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
        return linearVelocityMetersPerSec / radius;
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

        /**
         * Computes a ballistic solution by iterating angles from MIN to MAX hood angle,
         * finding the lowest angle that:
         *   1. Reaches the target
         *   2. Clears the funnel rim by DISTANCE_ABOVE_FUNNEL_TO_CLEAR
         * <p>
         * All heights are measured from the field floor.
         *
         * @param actualTarget   The real hub target (used for funnel radius scaling).
         * @param predictedTarget Used for shooting on the move: where we aim.
         * @return ProjectileData with exit velocity, hood angle, and target. Returns kNoSolution if no valid angle found.
         */
        public static ProjectileData calculateShot(
            Translation3d actualTarget, Translation3d predictedTarget
        ) {
            double x_dist = distanceToTarget(predictedTarget);
            double turretH = turretHeightAboveField();
            double y_dist = predictedTarget.getZ() - turretH;
            double r = FieldConstants.Hub.FUNNEL_RADIUS
                * x_dist / distanceToTarget(actualTarget);
            double xFunnel = x_dist - r;
            double requiredHeight = (FieldConstants.Hub.FUNNEL_HEIGHT
                + FieldConstants.Hub.DISTANCE_ABOVE_FUNNEL_TO_CLEAR)
                - turretH;

            double bestTheta = -1;
            double bestV0 = -1;

            for (double theta = RobotConstants.MIN_HOOD_ANGLE.getRadians();
                 theta <= RobotConstants.MAX_HOOD_ANGLE.getRadians();
                 theta += RobotConstants.ANGLE_INCREMENT
            ) {
                double cosTheta = Math.cos(theta);
                double tanTheta = Math.tan(theta);
                double denom = 2 * cosTheta * cosTheta * (x_dist * tanTheta - y_dist);
                if (denom <= 0) continue;

                double v0 = Math.sqrt(G * x_dist * x_dist / denom);
                if (Double.isNaN(v0) || Double.isInfinite(v0)) continue;
                double yAtFunnel = xFunnel * tanTheta
                    - (G * xFunnel * xFunnel) / (2.0 * v0 * v0 * cosTheta * cosTheta);

                if (yAtFunnel >= requiredHeight) {
                    bestTheta = theta;
                    bestV0 = v0;
                    break;
                }
            }

            if (bestTheta < 0) {
                if (Toggles.debugMode.get()) {
                    Logger.recordOutput("Turret/Debug/NoSolution", true);
                    Logger.recordOutput("Turret/Debug/x_dist", x_dist);
                    Logger.recordOutput("Turret/Debug/y_dist", y_dist);
                    Logger.recordOutput("Turret/Debug/requiredHeight", requiredHeight);
                    Logger.recordOutput("Turret/Debug/xFunnel", xFunnel);
                }
                return kNoSolution;
            }

            if (Toggles.debugMode.get()) {
                double cosB = Math.cos(bestTheta);
                double yAtFunnelFinal = xFunnel * Math.tan(bestTheta)
                    - (G * xFunnel * xFunnel) / (2.0 * bestV0 * bestV0 * cosB * cosB);
                Logger.recordOutput("Turret/Debug/x_dist", x_dist);
                Logger.recordOutput("Turret/Debug/y_dist", y_dist);
                Logger.recordOutput("Turret/Debug/xFunnel", xFunnel);
                Logger.recordOutput("Turret/Debug/requiredHeight", requiredHeight);
                Logger.recordOutput("Turret/Debug/yAtFunnel", yAtFunnelFinal);
                Logger.recordOutput("Turret/Debug/funnelClearance", yAtFunnelFinal - requiredHeight);
                Logger.recordOutput("Turret/Debug/bestTheta", Math.toDegrees(bestTheta));
                Logger.recordOutput("Turret/Debug/bestV0", bestV0);
            }
            return new ProjectileData(bestV0, bestTheta, predictedTarget);
        }

        /**
         * Computes a ballistic launch solution for a shooter with a fixed pitch angle.
         *
         * @param actualTarget   The real hub target.
         * @param predictedTarget Used for shooting on the move.
         * @return ProjectileData, or kNoSolution if the fixed angle cannot clear the funnel or reach the target.
         */
        public static ProjectileData calculateShotFixedPitch(
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
            double v0 = Math.sqrt(G * x * x / denom);
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
                solution = isFixedPitch
                    ? Static.calculateShotFixedPitch(actualTarget, predictedTarget)
                    : Static.calculateShot(actualTarget, predictedTarget);

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
