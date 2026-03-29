package org.steelhawks.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.parser.ParseException;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.Constants.Deadbands;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;

import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

    public static final SlewRateLimiter joystickLimiter = new SlewRateLimiter(0.8);
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static Translation2d lastVelocity = new Translation2d();

    private DriveCommands() {}

    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Deadbands.DRIVE_DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // square for more precise control
        linearMagnitude = Math.pow(linearMagnitude, 2);

        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
            () -> {
                Translation2d linearVelocity;
                if (RobotState.getInstance().getShootingState() == ShootingState.NOTHING) {
                    linearVelocity =
                        getLinearVelocityFromJoysticks(
                            Toggles.rateLimitSwerveEnabled.get()
                                ? joystickLimiter.calculate(xSupplier.getAsDouble())
                                : xSupplier.getAsDouble(),
                            Toggles.rateLimitSwerveEnabled.get()
                                ? joystickLimiter.calculate(ySupplier.getAsDouble())
                                : ySupplier.getAsDouble());
                    lastVelocity = linearVelocity;
                } else if (RobotState.getInstance().getShootingState() == ShootingState.SHOOTING_MOVING) {
                    double x = xSupplier.getAsDouble();
                    double y = ySupplier.getAsDouble();
                    double magnitude = Math.hypot(x, y);

                    if (magnitude >= Deadbands.DRIVE_DEADBAND) {
                        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
                        linearVelocity = new Pose2d(new Translation2d(), direction)
                            .transformBy(new Transform2d(0.5, 0.0, new Rotation2d()))
                            .getTranslation();
                        lastVelocity = linearVelocity;
                    }
                }
                double omega =
                    MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Deadbands.DRIVE_DEADBAND);
                omega = Math.copySign(Math.pow(omega, 2), omega);
                runVelocity(lastVelocity, omega);
            }, s_Swerve)
        .withName("Teleop Drive");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
        ProfiledPIDController alignController = s_Swerve.getAlign();

        return Commands.defer(() ->
            Commands.run(
            () -> {
                Rotation2d validatedTarget = AllianceFlip.apply(rotationSupplier.get());

                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        Toggles.rateLimitSwerveEnabled.get()
                            ? joystickLimiter.calculate(xSupplier.getAsDouble())
                            : xSupplier.getAsDouble(),
                        Toggles.rateLimitSwerveEnabled.get()
                            ? joystickLimiter.calculate(ySupplier.getAsDouble())
                            : ySupplier.getAsDouble());

                double omega =
                    alignController.calculate(
                        RobotState.getInstance().getRotation().getRadians(), validatedTarget.getRadians());

                runVelocity(linearVelocity, omega);
            }, s_Swerve)
                .beforeStarting(() -> alignController.reset(RobotState.getInstance().getRotation().getRadians()))
                    .withName("Align to Angle"),
            Set.of(s_Swerve));
    }

    public static void runVelocity(Translation2d linearVelocity, double omega) {
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * s_Swerve.getMaxLinearSpeedMetersPerSec() * s_Swerve.getSpeedMultiplier(),
                linearVelocity.getY() * s_Swerve.getMaxLinearSpeedMetersPerSec() * s_Swerve.getSpeedMultiplier(),
                omega * s_Swerve.getMaxAngularSpeedRadPerSec() * s_Swerve.getSpeedMultiplier());
        s_Swerve.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                AllianceFlip.shouldFlip()
                    ? RobotState.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                        : RobotState.getInstance().getRotation()));
    }

    /**
     * Command to drive to a specific position.
     *
     * @param target        The target position the robot should drive to. Make sure this is a blue alliance
     *                      pose if you want it to be flipped correctly.
     * @param constraints The constraints the robot should follow during this procedure
     * @param emergencyStop The supplier that will stop the command if it returns true.
     * @return The command to drive to the target position.
     */
    public static Command driveToPosition(Pose2d target, PathConstraints constraints, BooleanSupplier emergencyStop) {
        return AutoBuilder.pathfindToPose(target, constraints)
            .onlyWhile(() -> s_Swerve.shouldContinuePathfinding(emergencyStop))
            .beforeStarting(() -> s_Swerve.setPathfinding(true))
            .finallyDo(() -> s_Swerve.setPathfinding(false))
            .withName("Drive to Position");
    }

    /**
     * Command to drive to a specific position.
     *
     * @param target        The target position the robot should drive to. Make sure this is a blue alliance
     *                      pose if you want it to be flipped correctly.
     * @param emergencyStop The supplier that will stop the command if it returns true.
     * @return The command to drive to the target position.
     */
    public static Command driveToPosition(Pose2d target, BooleanSupplier emergencyStop) {
        return driveToPosition(target, AutonConstants.CONSTRAINTS, emergencyStop);
    }

    /**
     * Command to drive to a specific position. This command will not stop until it reaches the target
     * position.
     *
     * @param target The target position the robot should drive to. Make sure this is a blue alliance
     *               pose if you want it to be flipped correctly.
     * @param constraints The constraints the robot should follow during this procedure.
     * @return The command to drive to the target position.
     */
    public static Command driveToPosition(Pose2d target, PathConstraints constraints) {
        return driveToPosition(target, constraints, () -> false);
    }

    /**
     * Command to drive to a specific position. This command will not stop until it reaches the target
     * position.
     *
     * @param target The target position the robot should drive to. Make sure this is a blue alliance
     *               pose if you want it to be flipped correctly.
     * @return The command to drive to the target position.
     */
    public static Command driveToPosition(Pose2d target) {
        return driveToPosition(target, () -> false);
    }

    /**
     * Command to drive to a specific path.
     *
     * @param path          The path the robot should drive to and then follow.
     * @param emergencyStop The supplier that will stop the command if it returns true.
     * @return The command to drive to the target path.
     */
    public static Command driveToPath(PathPlannerPath path, BooleanSupplier emergencyStop) {
        return AutoBuilder.pathfindThenFollowPath(path, AutonConstants.CONSTRAINTS)
            .onlyWhile(() -> s_Swerve.shouldContinuePathfinding(emergencyStop))
                .beforeStarting(() -> s_Swerve.setPathfinding(true))
                    .finallyDo(() -> s_Swerve.setPathfinding(false))
                        .withName("Drive to Path");
    }

    /**
     * Command to drive to a specific path. This command will not stop until it reaches the target
     * path.
     *
     * @param path The path the robot should drive to and then follow.
     * @return The command to drive to the target path.
     */
    public static Command driveToPath(PathPlannerPath path) {
        return driveToPath(path, () -> false);
    }


    public static PathPlannerPath fromChoreo(String trajectoryName) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(trajectoryName);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Command to follow a path.
     *
     * @param path The path the robot should follow.
     * @return The command to follow the path.
     */
    public static Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path)
            .withName("Follow Path");
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Swerve drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(() -> drive.runDriveCharacterization(0.0), drive).withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                        double voltage = timer.get() * FF_RAMP_RATE;
                        drive.runDriveCharacterization(voltage);
                        velocitySamples.add(drive.getFFCharacterizationVelocity());
                        voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                        NumberFormat formatter = new DecimalFormat("#0.00000");
                        System.out.println("********** Drive FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                    }));
    }

    /**
     * Measures the robot's wheel radius by spinning in a circle.
     */
    public static Command wheelRadiusCharacterization(Swerve drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                        limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        state.positions = drive.getWheelRadiusCharacterizationPositions();
                        state.lastAngle = RobotState.getInstance().getRotation();
                        state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                            var rotation = RobotState.getInstance().getRotation();
                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                            state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }
                            double wheelRadius =
                                (state.gyroDelta * Swerve.DRIVE_BASE_RADIUS) / wheelDelta;

                            NumberFormat formatter = new DecimalFormat("#0.000");
                            System.out.println(
                                "********** Wheel Radius Characterization Results **********");
                            System.out.println(
                                "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                            System.out.println(
                                "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                            System.out.println(
                                "\tWheel Radius: "
                                    + formatter.format(wheelRadius)
                                    + " meters, "
                                    + formatter.format(Units.metersToInches(wheelRadius))
                                    + " inches");
                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
