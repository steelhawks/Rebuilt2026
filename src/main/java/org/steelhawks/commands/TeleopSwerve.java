package org.steelhawks.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotState;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

import static org.steelhawks.commands.DriveCommands.joystickLimiter;

public class TeleopSwerve extends Command {

    private static final LoggedTunableNumber driveKp =
        new LoggedTunableNumber("TeleopSwerve/DrivekP", 5.0);
    private static final LoggedTunableNumber driveKd =
        new LoggedTunableNumber("TeleopSwerve/DrivekD", 0.1);
    private static final LoggedTunableNumber angleKp =
        new LoggedTunableNumber("TeleopSwerve/AnglekP", 2.0);
    private static final LoggedTunableNumber angleKd =
        new LoggedTunableNumber("TeleopSwerve/AnglekD", 0.01);

    private static final LoggedTunableNumber maxMetersPerSec =
        new LoggedTunableNumber("TeleopSwerve/MaxMetersPerSec", 0.0);
    private static final LoggedTunableNumber maxMetersPerSecSq =
        new LoggedTunableNumber("TeleopSwerve/MaxMetersPerSecSq", 0.0);

    private static final LoggedTunableNumber maxRadiansPerSec =
        new LoggedTunableNumber("TeleopSwerve/MaxRadiansPerSec", 0.0);
    private static final LoggedTunableNumber maxRadiansPerSecSq =
        new LoggedTunableNumber("TeleopSwerve/MaxRadiansPerSecSq", 0.0);

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
    private final Trigger shootingOnTheMove = new Trigger(
        () -> RobotState.getInstance().getAimState().equals(RobotState.ShootingState.SHOOTING_MOVING));
    @AutoLogOutput
    private final Trigger inTrenchTrigger = new Trigger(this::inTrenchZone).debounce(0.3);
    @AutoLogOutput
    private final Trigger inBumpTrigger = new Trigger(this::inBumpZone).debounce(0.3);

    // PID Controllers
    private final ProfiledPIDController trenchController;
    private final ProfiledPIDController bumpController;
    private final ProfiledPIDController angleController;

    public boolean inTrenchZone() {
        return false;
    }

    private boolean inBumpZone() {
        return false;
    }

    public TeleopSwerve(
        Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        this.s_Swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;

        shootingOnTheMove.onTrue(setDriveState(DriveState.LOCK_SOTM)
            .alongWith(Commands.runOnce(() -> {
                sotmHeadingSnapshot = RobotState.getInstance().getRotation().getRadians();
                double x = xSupplier.getAsDouble();
                double y = ySupplier.getAsDouble();
                double magnitude = Math.hypot(x, y);
                sotmSpeedSnapshotNormalized = MathUtil.clamp(magnitude, 0.0, 1.0);
                sotmDirectionSnapshot = magnitude > Constants.Deadbands.DRIVE_DEADBAND
                    ? new Rotation2d(Math.atan2(y, x))
                    : new Rotation2d(sotmHeadingSnapshot); // fallback to robot heading if stick is near center
            })));
        inTrenchTrigger.onTrue(setDriveState(DriveState.TRENCH_ALIGN));
        inBumpTrigger.onTrue(setDriveState(DriveState.BUMP_ALIGN));
        shootingOnTheMove.negate().and(inTrenchTrigger.negate()).and(inBumpTrigger.negate())
            .onTrue(setDriveState(DriveState.NORMAL));

        trenchController =
            new ProfiledPIDController(
                driveKp.getAsDouble(),
                0.0,
                driveKd.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    maxMetersPerSec.getAsDouble(),
                    maxMetersPerSecSq.getAsDouble()));
        bumpController =
            new ProfiledPIDController(
                driveKp.getAsDouble(),
                0.0,
                driveKd.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    maxMetersPerSec.getAsDouble(),
                    maxMetersPerSecSq.getAsDouble()));
        angleController =
            new ProfiledPIDController(
                angleKp.getAsDouble(),
                0.0,
                angleKd.getAsDouble(),
                new TrapezoidProfile.Constraints(
                    maxRadiansPerSec.getAsDouble(),
                    maxRadiansPerSecSq.getAsDouble()));
        addRequirements(s_Swerve);
    }

    public static Command setDriveState(DriveState state) {
        return Commands.runOnce(() -> currentDriveState = state);
    }

    @Override
    public void execute() {
        if (angleKp.hasChanged(hashCode())
            || angleKd.hasChanged(hashCode())
        ) {
            angleController.setPID(angleKp.getAsDouble(), 0.0, angleKd.getAsDouble());
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
                linearVelocity =
                    new Translation2d(
                        linearVelocity.getX(),
                        trenchController.calculate(
                            RobotState.getInstance().getEstimatedPose().getY(),
                            middleOfTrenchClearanceY));
                double errorTo0 = Math.abs(MathUtil.angleModulus(currentRad - 0.0));
                double errorToPi = Math.abs(MathUtil.angleModulus(currentRad - Math.PI));
                omega = angleController.calculate(currentRad, errorTo0 < errorToPi ? 0.0 : Math.PI);
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