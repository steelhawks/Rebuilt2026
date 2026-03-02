package org.steelhawks.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.geometry.RobotFootprint;
import org.steelhawks.util.geometry.Boundary;

import java.util.function.DoubleSupplier;

import static org.steelhawks.commands.DriveCommands.joystickLimiter;

public class TeleopSwerve extends Command {

    private static final LoggedTunableNumber driveKp =
        new LoggedTunableNumber("TeleopSwerve/DrivekP", 1.0);
    private static final LoggedTunableNumber driveKd =
        new LoggedTunableNumber("TeleopSwerve/DrivekD", 0.1);
    private static final LoggedTunableNumber angleKp =
        new LoggedTunableNumber("TeleopSwerve/AnglekP", 0.5);
    private static final LoggedTunableNumber angleKd =
        new LoggedTunableNumber("TeleopSwerve/AnglekD", 0.0);

    private static final LoggedTunableNumber maxMetersPerSec =
        new LoggedTunableNumber("TeleopSwerve/MaxMetersPerSec", 3.0);
    private static final LoggedTunableNumber maxMetersPerSecSq =
        new LoggedTunableNumber("TeleopSwerve/MaxMetersPerSecSq", 4.0);

    private static final LoggedTunableNumber maxRadiansPerSec =
        new LoggedTunableNumber("TeleopSwerve/MaxRadiansPerSec", 15.0);
    private static final LoggedTunableNumber maxRadiansPerSecSq =
        new LoggedTunableNumber("TeleopSwerve/MaxRadiansPerSecSq", 20.0);

    private final Swerve s_Swerve;
    private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
    private double sotmHeadingSnapshot = 0.0;
    private double sotmSpeedSnapshotNormalized = 0.0;
    private Rotation2d sotmDirectionSnapshot = new Rotation2d();

    public enum DriveState {
        NORMAL,
        TRENCH_ALIGN,
        BUMP_ALIGN,
        LOCK_SOTM
    }

    private static DriveState currentDriveState = DriveState.NORMAL;

    @AutoLogOutput
    private final Trigger sotmTrigger = new Trigger(
        () -> RobotState.getInstance().getAimState()
            .equals(RobotState.ShootingState.SHOOTING_MOVING));
    @AutoLogOutput
    private final Trigger inTrenchTrigger;
    @AutoLogOutput
    private final Trigger inBumpTrigger;

    // PID Controllers
    private final ProfiledPIDController trenchController;
    private final ProfiledPIDController bumpController;
    private final ProfiledPIDController angleController;

    public TeleopSwerve(
        RobotFootprint footprint, Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        this.s_Swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;

        inTrenchTrigger = Boundary.asTrigger(
            "LeftTrench",
            () -> AllianceFlip.apply(FieldConstants.Trench.TRENCH_LEFT_TRIGGER_BOX),
                RobotState.getInstance()::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER)
        .or(Boundary.asTrigger(
            "RightTrench",
            () -> AllianceFlip.apply(FieldConstants.Trench.TRENCH_RIGHT_TRIGGER_BOX),
                RobotState.getInstance()::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER))
            .debounce(0.3)
            .onTrue(setDriveState(DriveState.TRENCH_ALIGN));

        inBumpTrigger = Boundary.asTrigger(
            () -> AllianceFlip.apply(new Rectangle2d(new Translation2d(), new Translation2d())),
                RobotState.getInstance()::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER)
            .debounce(0.3)
            .onTrue(setDriveState(DriveState.BUMP_ALIGN));

        sotmTrigger.onTrue(setDriveState(DriveState.LOCK_SOTM)
            .alongWith(Commands.runOnce(() -> sotmHeadingSnapshot = RobotState.getInstance().getRotation().getRadians())));
        sotmTrigger.negate().and(inTrenchTrigger.negate()).and(inBumpTrigger.negate())
            .onTrue(setDriveState(DriveState.NORMAL));

        trenchController =
            new ProfiledPIDController(
                driveKp.getAsDouble(),
                0.0,
                driveKd.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    maxMetersPerSec.getAsDouble(),
                    maxMetersPerSecSq.getAsDouble()));
        trenchController.setTolerance(0.05);
        bumpController =
            new ProfiledPIDController(
                driveKp.getAsDouble(),
                0.0,
                driveKd.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    maxMetersPerSec.getAsDouble(),
                    maxMetersPerSecSq.getAsDouble()));
        bumpController.setTolerance(0.05);
        angleController =
            new ProfiledPIDController(
                angleKp.getAsDouble(),
                0.0,
                angleKd.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    maxRadiansPerSec.getAsDouble(),
                    maxRadiansPerSecSq.getAsDouble()));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(0.05);
        addRequirements(s_Swerve);
    }

    public static Command overrideState() {
        return Commands.run(() -> currentDriveState = DriveState.NORMAL);
    }

    public static Command setDriveState(DriveState state) {
        return Commands.runOnce(() -> currentDriveState = state);
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous()) {
            s_Swerve.stopWithX();
        };

        if (angleKp.hasChanged(hashCode())
            || angleKd.hasChanged(hashCode())
        ) {
            angleController.setPID(angleKp.getAsDouble(), 0.0, angleKd.getAsDouble());
        }
        if (driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
            trenchController.setPID(driveKp.getAsDouble(), 0.0, driveKd.getAsDouble());
        }
        Translation2d linearVelocity =
            DriveCommands.getLinearVelocityFromJoysticks(
                Toggles.rateLimitSwerveEnabled.get()
                    ? joystickLimiter.calculate(xSupplier.getAsDouble())
                    : xSupplier.getAsDouble(),
                Toggles.rateLimitSwerveEnabled.get()
                    ? joystickLimiter.calculate(ySupplier.getAsDouble())
                    : ySupplier.getAsDouble());
        double omega =
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);
        double currentRad = RobotState.getInstance().getRotation().getRadians();
        omega = Math.copySign(Math.pow(omega, 2), omega);
        switch (currentDriveState) {
            case TRENCH_ALIGN -> {
                double middleOfTrenchClearanceY =
                    RobotState.getInstance().getEstimatedPose().getY() >= FieldConstants.FIELD_WIDTH / 2.0
                        ? FieldConstants.FIELD_WIDTH - (FieldConstants.Trench.TRENCH_WIDTH / 2.0)
                        : FieldConstants.Trench.TRENCH_WIDTH / 2.0;
                Logger.recordOutput("TeleopSwerve/Trench/SetpointTranslation", middleOfTrenchClearanceY);
                linearVelocity =
                    new Translation2d(
                        linearVelocity.getX(),
                        trenchController.calculate(
                            RobotState.getInstance().getEstimatedPose().getY(),
                            middleOfTrenchClearanceY));
                double errorTo0 = Math.abs(MathUtil.angleModulus(currentRad - 0.0));
                double errorToPi = Math.abs(MathUtil.angleModulus(currentRad - Math.PI));
                double setpoint = errorTo0 < errorToPi ? 0.0 : Math.PI;
                Logger.recordOutput("TeleopSwerve/Trench/SetpointAngle", setpoint);
                omega =
                    MathUtil.applyDeadband(angleController.calculate(currentRad, setpoint), Constants.Deadbands.ANGLE_DEADBAND);
            }
            case BUMP_ALIGN -> {
                double closestCornerAngle = Math.round(currentRad / (Math.PI / 4.0)) * (Math.PI / 4.0);
                if ((Math.round(closestCornerAngle / (Math.PI / 4.0)) % 2) == 0) {
                    closestCornerAngle += Math.PI / 4.0;
                }
                omega = angleController.calculate(currentRad, closestCornerAngle);
            }
            case LOCK_SOTM -> {
                double x = xSupplier.getAsDouble();
                double y = ySupplier.getAsDouble();
                double magnitude = Math.hypot(x, y);

                if (magnitude < Constants.Deadbands.DRIVE_DEADBAND) {
                    linearVelocity = new Translation2d();
                } else {
                    // Allow driver to steer and vary speed, but cap at the snapshotted speed
                    // so predictInterceptPoint doesn't get a wildly different velocity
                    double clampedMagnitude = Math.min(magnitude, sotmSpeedSnapshotNormalized);
                    Rotation2d direction = new Rotation2d(Math.atan2(y, x));
                    linearVelocity = new Pose2d(new Translation2d(), direction)
                        .transformBy(new Transform2d(clampedMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();
                }
                omega = angleController.calculate(currentRad, sotmHeadingSnapshot);
                var chassisSpeeds = s_Swerve.getChassisSpeeds();
                double actualMagnitude = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
                Logger.recordOutput("TeleopSwerve/HeadingSnapshot", sotmHeadingSnapshot);
                Logger.recordOutput("TeleopSwerve/SpeedSnapshotNormalized", sotmSpeedSnapshotNormalized);
                Logger.recordOutput("TeleopSwerve/ActualChassisSpeedMagnitude", actualMagnitude);
            }
        }
        DriveCommands.runVelocity(linearVelocity, omega);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
    }
}
