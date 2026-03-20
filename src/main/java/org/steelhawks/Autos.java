package org.steelhawks;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.commands.ShootingCommands;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import java.io.IOException;
import java.util.List;

@SuppressWarnings("unused")
public final class Autos {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Intake s_Intake = RobotContainer.s_Intake;
    private static final Indexer s_Indexer = RobotContainer.s_Indexer;

    private static final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser");

    private static final AutoFactory factory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose, // A function that returns the current robot pose
            s_Swerve::setPose, // A function that resets the current robot pose to the provided Pose2d
            s_Swerve::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            s_Swerve, // The drive subsystem
            ((trajectory, starting) -> {
                Pose2d[] poses = trajectory.getPoses();
                FieldConstants.FIELD_2D.getObject("Path").setPoses(List.of(poses));
                Logger.recordOutput("Odometry/Trajectory", poses);
            })
        );

    public enum Misalignment {
        NONE,
        ROTATION_CW,
        ROTATION_CCW,
        X_LEFT,
        X_RIGHT,
        Y_FORWARD,
        Y_BACKWARD,
        MULTIPLE
    }

    public static void testingBoard() {
        autoChooser.addOption("Nothing", Commands.none().withName("NOTHING_AUTO"));
        autoChooser.addOption("Jacob", Commands.none().withName("JACOB"));
    }

    public static void init() {
        /* ------------- Autons ------------- */

        autoChooser.addOption("4 Meter Test", fourMeterTest().cmd().withName(ChoreoTraj.FourMeterTest.name()));
        autoChooser.addOption("4 Meter Spin Test", fourMeterTestSpin().cmd().withName(ChoreoTraj.FourMeterSpinTest.name()));
        autoChooser.addOption("Center Path Test", centerPathTest().cmd().withName(ChoreoTraj.CenterPath.name()));
        autoChooser.addOption("Right Rebound Auton", rightRebound().cmd().withName(ChoreoTraj.RRebound.name()));
        autoChooser.addOption("Left Rebound Auton", leftRebound().cmd().withName(ChoreoTraj.LRebound.name()));
//        autoChooser.addOption("Outpost Trench Climb", outpostTrenchClimbAuto().cmd().withName("OutpostTrenchClimbAuto"));
//        autoChooser.addOption("Outpost Trench Climb", outpostTrenchClimbAuto().cmd().withName("OutpostTrenchClimbAuto"));

        if (Toggles.tuningMode.get()) {
            /* ------------- Swerve SysId ------------- */

            autoChooser.addOption("Swerve Drive (Quick Characterizer)", DriveCommands.feedforwardCharacterization(s_Swerve));

            autoChooser.addOption("Swerve Drive (Quasistatic Forward)", s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Drive (Quasistatic Backward)", s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Drive (Dynamic Forward)", s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Drive (Dynamic Backward)", s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Turn (Quasistatic Forward)", s_Swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Turn (Quasistatic Backward)", s_Swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Turn (Dynamic Forward)", s_Swerve.turnSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Turn (Dynamic Backward)", s_Swerve.turnSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Angular (Quasistatic Forward)", s_Swerve.angularSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Angular (Quasistatic Backward)", s_Swerve.angularSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Angular (Dynamic Forward)", s_Swerve.angularSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Angular (Dynamic Backward)", s_Swerve.angularSysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
    }

    public static Misalignment getMisalignment() {
        if (Toggles.tuningMode.get()) {
            return Misalignment.NONE;
        }
        String autoName = getAuto().getName();
        ChoreoTraj trajectory = ChoreoTraj.ALL_TRAJECTORIES.get(autoName);
        if (trajectory == null) {
            return Misalignment.NONE;
        }
        double radiansTolerance = Units.degreesToRadians(5);
        double xyTolerance = 0.6;

        double rotError = AllianceFlip.apply(trajectory.initialPoseBlue().getRotation()).getRadians() - RobotState.getInstance().getRotation().getRadians();
        double xError = AllianceFlip.applyX(trajectory.initialPoseBlue().getX()) - RobotState.getInstance().getEstimatedPose().getX();
        double yError = AllianceFlip.applyY(trajectory.initialPoseBlue().getY()) - RobotState.getInstance().getEstimatedPose().getY();

        boolean rotAligned = Math.abs(rotError) <= radiansTolerance;
        boolean xAligned = Math.abs(xError) <= xyTolerance;
        boolean yAligned = Math.abs(yError) <= xyTolerance;

        Logger.recordOutput(autoName + "/OmegaAligned", rotAligned);
        Logger.recordOutput(autoName + "/XAligned", xAligned);
        Logger.recordOutput(autoName + "/YAligned", yAligned);

        if (rotAligned && xAligned && yAligned) {
            return Misalignment.NONE;
        }
        if (!rotAligned && !xAligned && !yAligned) {
            return Misalignment.MULTIPLE;
        }
        if (!xAligned) {
            return (xError > 0) ? Misalignment.X_RIGHT : Misalignment.X_LEFT;
        }
        if (!yAligned) {
            return (yError > 0) ? Misalignment.Y_FORWARD : Misalignment.Y_BACKWARD;
        }
        return (rotError > 0) ? Misalignment.ROTATION_CCW : Misalignment.ROTATION_CW; // omega not being aligned is final scenario
    }

    public static Command followTrajectory(String pathPlanner) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathPlanner);
            return DriveCommands.followPath(path).withName("Following " + pathPlanner);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static Command getAuto() {
        return autoChooser.get();
    }

    public static AutoRoutine fourMeterTest() {
        AutoRoutine routine = factory.newRoutine("4 Meter Test");

        AutoTrajectory start = ChoreoTraj.FourMeterTest.asAutoTraj(routine);

        routine.active().onTrue(
            start.cmd()
        );

        return routine;
    }

    public static AutoRoutine fourMeterTestSpin() {
        AutoRoutine routine = factory.newRoutine("4 Meter Spin Test");

        AutoTrajectory start = ChoreoTraj.FourMeterSpinTest.asAutoTraj(routine);

        routine.active().onTrue(
            start.cmd()
        );

        return routine;
    }

    public static AutoRoutine centerPathTest() {
        AutoRoutine routine = factory.newRoutine("Center Path Test");

        AutoTrajectory start = ChoreoTraj.CenterPath$0.asAutoTraj(routine);
        AutoTrajectory back = ChoreoTraj.CenterPath$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                ShootingCommands.shoot().withTimeout(5.0),
//                start.resetOdometry(),
                start.cmd()
                    .alongWith(RobotContainer.s_Intake.runIntake().withTimeout(5.0)),
                back.cmd(),
                ShootingCommands.shoot()));

        return routine;
    }

    public static AutoRoutine rightRebound() {
        AutoRoutine routine = factory.newRoutine("Right Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.RRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.RRebound$1.asAutoTraj(routine);
//        AutoTrajectory trenchToOutpost = ChoreoTraj.RRebound$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
//                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
                new ScheduleCommand(RobotContainer.s_Intake.slamOut()),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                ShootingCommands.shoot().withTimeout(2.0), // TODO tune
                ShootingCommands.shoot().until(() -> !s_Indexer.hasBalls()),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                ShootingCommands.shoot().withTimeout(2.0),
                ShootingCommands.shoot().until(() -> !s_Indexer.hasBalls()),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
//                trenchToOutpost.spawnCmd()
            )
        );
//
//        trenchToOutpost.done().onTrue(
//            Commands.sequence(
//                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
//                ShootingCommands.shoot().withTimeout(5.0)
//            )
//        );

        return routine;
    }

    public static AutoRoutine leftRebound() {
        AutoRoutine routine = factory.newRoutine("Left Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.LRebound$1.asAutoTraj(routine);
        AutoTrajectory trenchToOutpost = ChoreoTraj.LRebound$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
//                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
                new ScheduleCommand(RobotContainer.s_Intake.slamOut()),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
//                ShootingCommands.shoot().withTimeout(2.0), // TODO tune
//                ShootingCommands.shoot().until(() -> !s_Indexer.hasBalls()),
                ShootingCommands.shoot().withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                ShootingCommands.shoot().withTimeout(5.0),
//                ShootingCommands.shoot().until(() -> !s_Indexer.hasBalls()),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToOutpost.spawnCmd()
            )
        );

        trenchToOutpost.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                ShootingCommands.shoot().withTimeout(5.0)
            )
        );

        return routine;
    }

//    public static AutoRoutine rightReboundWithBump() {
//        AutoRoutine routine = factory.newRoutine("Right Rebound With Bump Auton");
//
//        AutoTrajectory trenchToMidToHub = ChoreoTraj.RRebound_Bump$0.asAutoTraj(routine);
//        AutoTrajectory hubToReboundToTrench = ChoreoTraj.RRebound_Bump$1.asAutoTraj(routine);
//        AutoTrajectory trenchToOutpost = ChoreoTraj.RRebound_Bump$2.asAutoTraj(routine);
//
//        routine.active().onTrue(
//            Commands.sequence(
//                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE);
//                trenchToMidToHub
//            )
//        )
//    }

    public static Command followTrajectory(ChoreoTraj traj) {
        return factory.trajectoryCmd(traj.name());
    }

//    public static AutoRoutine outpostTrenchClimbAuto() {
//        AutoRoutine routine = factory.newRoutine("outpostTrenchClimbAuto");
//
//        AutoTrajectory startToOutpost = ChoreoTraj.OutpostTrenchClimbAuto.segment(0).asAutoTraj(routine);
//        AutoTrajectory outpostToTrench = ChoreoTraj.OutpostTrenchClimbAuto.segment(1).asAutoTraj(routine);
//        AutoTrajectory trenchToClimb = ChoreoTraj.OutpostTrenchClimbAuto.segment(2).asAutoTraj(routine);
//
//        routine.active().onTrue(
//            Commands.sequence(
//                startToOutpost.resetOdometry(),
//                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
//                startToOutpost.cmd()
//                    .alongWith(RobotContainer.s_Intake.runIntake().withTimeout(2.0)),
//                Commands.waitSeconds(1.0),
//                outpostToTrench.cmd()
//                    .alongWith(RobotContainer.s_Intake.runIntake().withTimeout(2.0)),
//                Commands.waitSeconds(0.5),
//                trenchToClimb.cmd(),
//                Commands.sequence(
//                RobotContainer.s_Flywheel.testfire(),
//                    Commands.waitSeconds(0.25)
//                ).repeatedly().withTimeout(4.0)
//            )
//        );
//
//        return routine;
//    }
}
