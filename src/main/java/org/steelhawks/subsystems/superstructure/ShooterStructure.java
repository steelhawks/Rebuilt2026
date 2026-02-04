package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;

public class ShooterStructure {

    public record ProjectileData(double exitVelocity, double hoodAngle, Translation3d target) {}
    private static ShooterMode currentMode = ShooterMode.TO_HUB;
    private static final double G = 9.81;

    public enum ShooterMode {
        TO_HUB,
        FERRY,
        MANUAL
    }

    public static void setMode(ShooterMode mode) {
        if (currentMode != mode) {
            Logger.recordOutput("SuperStructure/ModeChange",
                currentMode.name() + " -> " + mode.name());
            currentMode = mode;
            Logger.recordOutput("SuperStructure/CurrentMode", mode.name());
            // clear trajectory
            Logger.recordOutput("Turret/ScoreTrajectory", new Translation3d[0]);
            Logger.recordOutput("Turret/FerryTrajectory", new Translation3d[0]);
        }
    }

    public static ShooterMode getMode() {
        return currentMode;
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
        return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(target.toTranslation2d());
    }

    public static class Static {

        /**
         * Computes a ballistic solution by constructing a quadratic that passes through the shooter, clears the funnel by a height, and reaches the target.
         *
         * <br/>
         * @param actualTarget The target you want to aim at, usually the Hub.
         * @param predictedTarget Used for Shooting on the Move: reiteration and look-ahead.
         * @return Projectile data: initial velocity, theta, and target
         *
         * <a href="https://www.desmos.com/calculator/ezjqolho6g">Desmos Graph</a>
         */
        public static ProjectileData calculateShot(
            Translation3d actualTarget, Translation3d predictedTarget
        ) {
            double x_dist = distanceToTarget(predictedTarget);
            double y_dist = predictedTarget.getZ() - RobotConstants.ROBOT_TO_TURRET.getZ();
            double r = FieldConstants.Hub.FUNNEL_RADIUS * x_dist
                / distanceToTarget(actualTarget);
            double h = FieldConstants.Hub.FUNNEL_HEIGHT + FieldConstants.Hub.DISTANCE_ABOVE_FUNNEL_TO_CLEAR;
            double A1 = Math.pow(x_dist, 2);
            double B1 = x_dist;
            double D1 = y_dist;
            double A2 = -x_dist * x_dist + Math.pow((x_dist - r), 2);
            double B2 = -r;
            double D2 = h;
            double Bm = -B2 / B1;
            double A3 = Bm * A1 + A2;
            double D3 = Bm * D1 + D2;
            double a = D3 / A3;
            double b = (D1 - A1 * a) / B1;
            double theta = Math.atan(b);
            double v0 = Math.sqrt(-G / (2 * a * Math.pow(Math.cos(theta), 2)));
            return new ProjectileData(v0, theta, predictedTarget);
        }

        /**
         * Computes a ballistic launch solution for a shooter with a fixed pitch angle using ideal kinematic projectile motion.
         *
         * <p>
         * If the shot is physically impossible (for example, the fixed angle is too
         * shallow to reach the target height, or the trajectory intersects the funnel),
         * this method returns {@code null}.
         * </p>
         *
         * <br/>
         * @param actualTarget The target you want to aim at, usually the Hub.
         * @param predictedTarget Used for Shooting on the Move: reiteration and look-ahead.
         * @return Projectile data: initial velocity, theta, and target
         * <br/>
         * <br/> NOTE: Theta will be the same theta set in ({@code RobotConstants.FIXED_SHOOTER_ANGLE})
         */
        public static ProjectileData calculateShotFixedPitch(
            Translation3d actualTarget, Translation3d predictedTarget
        ) {
            final double theta = RobotConstants.FIXED_SHOOTER_ANGLE;
            double x = distanceToTarget(predictedTarget);
            double y = predictedTarget.getZ()
                - RobotConstants.ROBOT_TO_TURRET.getZ();
            double denom = 2 * Math.pow(Math.cos(theta), 2)
                * (x * Math.tan(theta) - y);
            if (denom <= 0) { // impossible at 45 deg or wtv angle at
                return null;
            }
            double v0 = Math.sqrt(G * Math.pow(x, 2) / denom);

            // funnel clearance checks
            double r = FieldConstants.Hub.FUNNEL_RADIUS * x
                / distanceToTarget(actualTarget);
            double xFunnel = x - r;
            double requiredHeight =
                FieldConstants.Hub.FUNNEL_HEIGHT
                    + FieldConstants.Hub.DISTANCE_ABOVE_FUNNEL_TO_CLEAR;
            double yAtFunnel =
                xFunnel * Math.tan(theta)
                    - (G * Math.pow(xFunnel, 2))
                    / (2 * v0 * v0 * Math.pow(Math.cos(theta), 2));

            if (yAtFunnel < requiredHeight) { // will hit funnel
                return null;
            }
            return new ProjectileData(v0, theta, predictedTarget);
        }

    }

    public static class Moving {

        private static final int MAX_ITERATIONS = 5;
        private static final double CONVERGENCE_THRESHOLD = 0.01; // meters

        /**
         * Calculates shot parameters for shooting while moving. Uses ToF iteration to predict
         * where the target will be when the projectile arrives.
         *
         * <a href="https://www.chiefdelphi.com/t/time-of-flight-determination/512542">Read more</a>
         * @param actualTarget The actual target you want to score at: Hub, Ferry.
         * @param isFixedPitch If this is a fixed pitch shooter or not.
         * @return ProjectileData with exit velocity, hood angle, and predicted intercept point.
         */
        public static ProjectileData calculateMovingShot(
            Translation3d actualTarget,
            boolean isFixedPitch
        ) {
            return calculateMovingShot(actualTarget, new Translation3d(), isFixedPitch);
        }

        /**
         * Calculates shot parameters for shooting while moving. Uses ToF iteration to predict
         * where the target will be when the projectile arrives.
         *
         * <a href="https://www.chiefdelphi.com/t/time-of-flight-determination/512542">Read more</a>
         * @param actualTarget The actual target you want to score at: Hub, Ferry.
         * @param targetVelocity The velocity the target is moving at.
         * @param isFixedPitch If this is a fixed pitch shooter or not.
         * @return ProjectileData with exit velocity, hood angle, and predicted intercept point.
         */
        public static ProjectileData calculateMovingShot(
            Translation3d actualTarget,
            Translation3d targetVelocity,
            boolean isFixedPitch
        ) {
            Translation3d robotVelocity =
                new Translation3d(
                    RobotContainer.s_Swerve.getChassisSpeeds().vxMetersPerSecond,
                    RobotContainer.s_Swerve.getChassisSpeeds().vyMetersPerSecond,
                    0.0);
            Translation3d predictedTarget = actualTarget;
            ProjectileData solution = null;
            for (int i = 0; i < MAX_ITERATIONS; i++) {
                solution = isFixedPitch
                    ? Static.calculateShotFixedPitch(actualTarget, predictedTarget)
                    : Static.calculateShot(actualTarget, predictedTarget);
                double distance = distanceToTarget(predictedTarget);
                if (solution == null) {
                    Logger.recordOutput("Shooter/Moving/ShotImpossible", true);
                    return null;
                }
                double tof =
                    calculateTimeofFlight(
                        solution.exitVelocity(), solution.hoodAngle(), distance);
                Translation3d relativeVelocity = targetVelocity.minus(robotVelocity);
                Translation3d newPredictedTarget =
                    actualTarget.plus(relativeVelocity.times(tof));

                // check error
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
