package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation3d;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;

public class ShooterPhysics {

    private static final double G = 9.81;

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
        return RobotContainer.s_Swerve.getPose().getTranslation().getDistance(target.toTranslation2d());
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
            double r = FieldConstants.FUNNEL_RADIUS * x_dist
                / distanceToTarget(actualTarget);
            double h = FieldConstants.FUNNEL_HEIGHT + FieldConstants.DISTANCE_ABOVE_FUNNEL_TO_CLEAR;
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
            double r = FieldConstants.FUNNEL_RADIUS * x
                / distanceToTarget(actualTarget);
            double xFunnel = x - r;
            double requiredHeight =
                FieldConstants.FUNNEL_HEIGHT
                    + FieldConstants.DISTANCE_ABOVE_FUNNEL_TO_CLEAR;
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

    }

    public record ProjectileData(double exitVelocity, double hoodAngle, Translation3d target) {}
}

