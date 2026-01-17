package org.steelhawks.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.RobotContainer;

import java.util.function.Supplier;

public class ChaseTagCommand extends Command {

    private static final double X_KP = 3;
    private static final double X_KI = 0;
    private static final double X_KD = 0;
    private static final double X_TOLERANCE = 0.2;

    private static final double Y_KP = 3;
    private static final double Y_KI = 0;
    private static final double Y_KD = 0;
    private static final double Y_TOLERANCE = 0.2;

    private static final double OMEGA_KP = 2;
    private static final double OMEGA_KI = 0;
    private static final double OMEGA_KD = 0;
    private static final double OMEGA_TOLERANCE = Units.degreesToRadians(3);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final int idToChase;
    private final int cameraIndex;
    private static final Transform3d TAG_TO_GOAL =
        new Transform3d(
            new Translation3d(1.5 , 0.0, 0.0), // stay 1.5 meters away from the tag
            new Rotation3d(0.0, 0.0, Math.PI));

    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController omegaController;

//    private PhotonTrackedTarget mLastTarget;
    private Pose2d mLastTarget;

    public ChaseTagCommand(int idToChase, int cameraIndex, Supplier<Pose2d> poseProvider) {
        this.idToChase = idToChase;
        this.cameraIndex = cameraIndex;
        this.poseProvider = poseProvider;

        xController =
            new ProfiledPIDController(X_KP, X_KI, X_KD, X_CONSTRAINTS);
        yController =
            new ProfiledPIDController(Y_KP, Y_KI, Y_KD, Y_CONSTRAINTS);
        omegaController =
            new ProfiledPIDController(OMEGA_KP, OMEGA_KI, OMEGA_KD, OMEGA_CONSTRAINTS);

        xController.setTolerance(X_TOLERANCE);
        yController.setTolerance(Y_TOLERANCE);
        omegaController.setTolerance(OMEGA_TOLERANCE);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(RobotContainer.s_Swerve);
    }

    @Override
    public void initialize() {
        mLastTarget = null;
        var robotPose = poseProvider.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose =
            new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        Rotation2d targetX = RobotContainer.s_Vision.getTargetX(cameraIndex);
        Rotation2d targetY = RobotContainer.s_Vision.getTargetY(cameraIndex);

//        Pose2d goalPose = mLastTarget.rotateBy()

//        xController.setGoal(goalPose.getX());
//        yController.setGoal(goalPose.getY());
//        omegaController.setGoal(goalPose.getRotation().getRadians);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
