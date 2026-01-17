package org.steelhawks.commands.align;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.AutonConstants;
import org.steelhawks.FieldConstants;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class SwerveDriveAlignment extends Command {

    protected static final double XY_TOLERANCE = Units.inchesToMeters(1.0); // m
    protected static final double THETA_TOLERANCE = Units.degreesToRadians(3); // rad/s
    protected static final double MAX_VELOCITY_ERROR_TOLERANCE = 0.15; // m/s

    /* All these are math estimated values, could do more real life regressions later */
    private static final double MAX_ACCELERATION = 20.0; // m/s^2
    private static final double MAX_JERK = 15.0; // m/s^3
    private static final double BEELINE_RADIUS_TELEOP = 10.0; // cm
    // drive in straight line in auton, do not curve at any distance
    private static final double BEELINE_RADIUS_AUTON = Double.POSITIVE_INFINITY; // cm

    protected static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final APConstraints CONSTRAINTS = new APConstraints()
        .withAcceleration(MAX_ACCELERATION)
        .withJerk(MAX_JERK);

    private final ProfiledPIDController angleController;
    private final APProfile profile;
    private final Autopilot autopilot;
    private APTarget target = null;

    protected final FieldObject2d dashboardTargetPosePublisher;
    protected Supplier<Pose2d> targetPose;
    protected final Debouncer debouncer;
    protected final LinearFilter filter;
    protected Pose2d startingPose;
    protected double velocityError;

    private final boolean endsWhenAligned;

    public SwerveDriveAlignment(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public SwerveDriveAlignment(Pose2d targetPose, boolean endsWhenAligned) {
        this(() -> targetPose, endsWhenAligned);
    }

    public SwerveDriveAlignment(Supplier<Pose2d> targetPose) {
        this(targetPose, false);
    }

    public SwerveDriveAlignment(Supplier<Pose2d> targetPose, boolean endsWhenAligned) {
        addRequirements(s_Swerve);
        dashboardTargetPosePublisher = FieldConstants.FIELD_2D.getObject("Trajectory Setpoint");
        this.targetPose = targetPose;
        this.debouncer = new Debouncer(0.2, Debouncer.DebounceType.kRising);
        this.filter = LinearFilter.movingAverage(5);
        this.endsWhenAligned = endsWhenAligned;

        angleController =
            new ProfiledPIDController(
                AutonConstants.ROTATION_KP.get(),
                AutonConstants.ROTATION_KI.get(),
                AutonConstants.ROTATION_KD.get(),
                new TrapezoidProfile.Constraints(
                    AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
        profile = new APProfile(CONSTRAINTS)
            .withErrorXY(Meters.of(XY_TOLERANCE))
            .withErrorTheta(Radians.of(THETA_TOLERANCE))
            .withBeelineRadius(Centimeters.of(BEELINE_RADIUS_TELEOP));
        autopilot = new Autopilot(profile);
    }

    protected boolean isXAligned() {
        return Math.abs(targetPose.get().getX() - s_Swerve.getPose().getX()) <= XY_TOLERANCE;
    }

    protected boolean isYAligned() {
        return Math.abs(targetPose.get().getY() - s_Swerve.getPose().getY()) <= XY_TOLERANCE;
    }

    protected boolean isThetaAligned() {
        double angleDifference = MathUtil.angleModulus(targetPose.get().getRotation().getRadians() - s_Swerve.getPose().getRotation().getRadians());
        return Math.abs(angleDifference) <= THETA_TOLERANCE;
    }

    protected boolean velocityInTolerance() {
        return Math.abs(filter.calculate(velocityError)) < MAX_VELOCITY_ERROR_TOLERANCE;
    }

    protected boolean isAligned() {
        return autopilot.atTarget(s_Swerve.getPose(), new APTarget(targetPose.get()));
    }

    @Override
    public void initialize() {
        startingPose = s_Swerve.getPose();
        angleController.reset(startingPose.getRotation().getRadians());
//        autopilot.calculate(s_Swerve.getPose(), s_Swerve.getChassisSpeeds(), new APTarget(s_Swerve.getPose()));
        s_Swerve.setPathfinding(true);
    }

    /**
     * Runs the PID loop and returns a filtered output with a deadband to stop wheel oscillation.
     *
     * @return The ChassisSpeeds output
     */
    protected ChassisSpeeds getOutput() {
        ChassisSpeeds robotRelativeSpeeds = s_Swerve.getChassisSpeeds();
        Pose2d currPose = s_Swerve.getPose();
        target = new APTarget(targetPose.get())
            .withEntryAngle(Rotation2d.kZero);
        APResult output = autopilot.calculate(currPose, robotRelativeSpeeds, target);
        double vx = output.vx().in(MetersPerSecond);
        double vy = output.vy().in(MetersPerSecond);
        double omegaRadPerSec = angleController.calculate(currPose.getRotation().getRadians(), output.targetAngle().getRadians());

        Logger.recordOutput("Align/SwerveAlign/VelocitySetpointX", vx);
        Logger.recordOutput("Align/SwerveAlign/VelocitySetpointY", vy);
        Logger.recordOutput("Align/SwerveAlign/ControllerOutputTheta", omegaRadPerSec);

        boolean nearZeroTrans =
            Math.abs(vx) < XY_TOLERANCE
                && Math.abs(vy) < XY_TOLERANCE;
        boolean nearZeroRot =
            Math.abs(omegaRadPerSec)
                < THETA_TOLERANCE;
        if (nearZeroTrans && nearZeroRot)
            return new ChassisSpeeds();

        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadPerSec, currPose.getRotation());
    }

    protected void log() {
        dashboardTargetPosePublisher.setPose(targetPose.get());
        var speeds = s_Swerve.getChassisSpeeds();
        velocityError =
            target.getVelocity() -
                Math.hypot(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond);
        Logger.recordOutput("Align/SwerveAlign/VelocityError", velocityError);

        Logger.recordOutput("Align/SwerveAlign/TargetX", targetPose.get().getX());
        Logger.recordOutput("Align/SwerveAlign/TargetY", targetPose.get().getY());
        Logger.recordOutput("Align/SwerveAlign/TargetTheta", targetPose.get().getRotation().getDegrees());
        Logger.recordOutput("Align/SwerveAlign/CurrentX", s_Swerve.getPose().getX());
        Logger.recordOutput("Align/SwerveAlign/CurrentY", s_Swerve.getPose().getY());
        Logger.recordOutput("Align/SwerveAlign/CurrentTheta", s_Swerve.getPose().getRotation().getDegrees());

        Logger.recordOutput("Align/SwerveAlign/TargetPose", targetPose.get());
        Logger.recordOutput("Align/SwerveAlign/StartingPose", startingPose);

        Logger.recordOutput("Align/SwerveAlign/AlignedX", isXAligned());
        Logger.recordOutput("Align/SwerveAlign/AlignedY", isYAligned());
        Logger.recordOutput("Align/SwerveAlign/AlignedTheta", isThetaAligned());
        Logger.recordOutput("Align/SwerveAlign/VelocityInTolerance", velocityInTolerance());
    }

    protected void updatePID() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            angleController.setPID(
                AutonConstants.ROTATION_KP.get(),
                AutonConstants.ROTATION_KI.get(),
                AutonConstants.ROTATION_KD.get());
        },
            AutonConstants.ROTATION_KP,
            AutonConstants.ROTATION_KI,
            AutonConstants.ROTATION_KD);
    }

    private void updateProfile() {
        if (Robot.getState().equals(Robot.RobotState.AUTON)) {
            profile.withBeelineRadius(Centimeters.of(BEELINE_RADIUS_AUTON));
        } else {
            profile.withBeelineRadius(Centimeters.of(BEELINE_RADIUS_TELEOP));
        }
    }

    @Override
    public void execute() {
        s_Swerve.runVelocity(getOutput());
        updateProfile();
        updatePID();
        log();
    }

    @Override
    public boolean isFinished() {
        // added auton check so command keeps running if the driver wants to switch the branch to score on, this doesnt interrupt auton scoring sequence
        return debouncer.calculate(isAligned()) && (Robot.getState() == Robot.RobotState.AUTON || endsWhenAligned);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.runVelocity(new ChassisSpeeds());
        s_Swerve.setPathfinding(false);
    }
}
