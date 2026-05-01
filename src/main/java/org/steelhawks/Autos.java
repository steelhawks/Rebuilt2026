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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.commands.DriveCommands;
import org.steelhawks.commands.ShootingCommands;
import org.steelhawks.commands.align.SwerveDriveAlignment;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.Intake.State;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

@SuppressWarnings("unused")
public final class Autos {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Intake s_Intake = RobotContainer.s_Intake;
    private static final Indexer s_Indexer = RobotContainer.s_Indexer;

    private static final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser");

    private static final AutoFactory factory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose,
            s_Swerve::setPose,
            s_Swerve::followTrajectory,
            true,
            s_Swerve,
            ((trajectory, starting) -> {
                Pose2d[] poses = Arrays.stream(trajectory.getPoses())
                    .map(AllianceFlip::apply)
                    .toArray(Pose2d[]::new);
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

    // Distance threshold to decide between pathfinding vs simple PID recovery
    public static double replanDistanceRequirement = Units.inchesToMeters(5.0); // tune
    private static boolean tuningOptionsAdded = false;

    /**
     * After a trajectory finishes, checks if the robot was bumped away from the
     * trajectory's end pose and recovers accordingly:
     * Call this AFTER a trajectory ends, BEFORE shooting or spawning the next trajectory.
     */
    public static Command recoverToTrajectoryEnd(AutoTrajectory traj) {
        return Commands.defer(() -> {
            Pose2d finalPose = traj.getFinalPose()
                .orElse(RobotState.getInstance().getEstimatedPose());
            double distanceFromEnd = RobotState.getInstance().getEstimatedPose()
                .getTranslation()
                .getDistance(finalPose.getTranslation());

            if (distanceFromEnd >= replanDistanceRequirement) {
                return DriveCommands.driveToPosition(finalPose);
            } else {
                return new SwerveDriveAlignment(finalPose, true).withTimeout(1.0);
            }
        }, Set.of(s_Swerve));
    }

    public static void init() {
        autoChooser.addOption("No Auton", Commands.none().withName("No Auton"));
//        autoChooser.addOption("4 Meter Test", fourMeterTest().cmd().withName(ChoreoTraj.FourMeterTest.name()));
//        autoChooser.addOption("4 Meter Spin Test", fourMeterTestSpin().cmd().withName(ChoreoTraj.FourMeterSpinTest.name()));
//        autoChooser.addOption("Center Path Test", centerPathTest().cmd().withName(ChoreoTraj.CenterPath.name()));
        autoChooser.addOption("Right Rebound Auton", rightRebound().cmd().withName(ChoreoTraj.RRebound.name()));
        autoChooser.addOption("Left Rebound Auton", leftRebound().cmd().withName(ChoreoTraj.LRebound.name()));
        autoChooser.addOption("Left Rebound Auton Q112", leftRebound112().cmd().withName(ChoreoTraj.LRebound.name()));
        autoChooser.addOption("Right Double Rebound Auton", rightDoubleRebound().cmd().withName(ChoreoTraj.RDoubleRebound.name()));
        autoChooser.addOption("Left Double Rebound Auton", leftDoubleRebound().cmd().withName(ChoreoTraj.LDoubleRebound.name()));
        autoChooser.addOption("Right OP Auton", rightOP().cmd().withName(ChoreoTraj.ROPAuton.name()));
        autoChooser.addOption("Left OP Auton", leftOP().cmd().withName(ChoreoTraj.LOPAuton.name()));
        autoChooser.addOption("Right Not So OP Auton", rightNotSoOP().cmd().withName(ChoreoTraj.RNotSoOPAuton.name()));
        autoChooser.addOption("Left Not So OP Auton", leftNotSoOP().cmd().withName(ChoreoTraj.LNotSoOPAuton.name()));
        autoChooser.addOption("Middle Depot Auton", middleDepotAuton().cmd().withName(ChoreoTraj.MiddleDepotAuton.name()));
        autoChooser.addOption("Middle Depot Auton 2", middleDepotAuton2().cmd().withName(ChoreoTraj.MiddleDepotAuton2.name()));
        autoChooser.addOption("Right Stuy Auton", rightStuy().cmd().withName(ChoreoTraj.RStuyAuton.name()));
        autoChooser.addOption("Left Stuy Auton", leftStuy().cmd().withName(ChoreoTraj.LStuyAuton.name()));
        autoChooser.addOption("Right Bump Hub Depot Auton", rightBumpHubDepot().cmd().withName(ChoreoTraj.RBumpHubDepot.name()));
        autoChooser.addOption("Left Bump Hub Depot Auton", leftBumpHubDepot().cmd().withName(ChoreoTraj.LBumpHubDepot.name()));
        autoChooser.addOption("Stationary Shoot", ShootingCommands.shoot());

        if (Toggles.tuningMode.get()) {
            pollTuningMode();
        }
    }

    public static void pollTuningMode() {
        if (!tuningOptionsAdded && Toggles.tuningMode.get()) {

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

            autoChooser.addOption("Flywheel (Quick Characterizer)", RobotContainer.s_Flywheel.feedforwardCharacterization());
            autoChooser.addOption("Flywheel (Quasistatic Forward)", RobotContainer.s_Flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Flywheel (Quasistatic Backward)", RobotContainer.s_Flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Flywheel (Dynamic Forward)", RobotContainer.s_Flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Flywheel (Quasistatic Backward)", RobotContainer.s_Flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Turret (Quick Characterizer)", RobotContainer.s_Turret.feedforwardCharacterization());
            autoChooser.addOption("Hood (Quick Characterizer)", RobotContainer.s_Hood.feedforwardCharacterization());
            tuningOptionsAdded = true;
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

        if (rotAligned && xAligned && yAligned) return Misalignment.NONE;
        if (!rotAligned && !xAligned && !yAligned) return Misalignment.MULTIPLE;
        if (!xAligned) return (xError > 0) ? Misalignment.X_RIGHT : Misalignment.X_LEFT;
        if (!yAligned) return (yError > 0) ? Misalignment.Y_FORWARD : Misalignment.Y_BACKWARD;
        return (rotError > 0) ? Misalignment.ROTATION_CCW : Misalignment.ROTATION_CW;
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
        routine.active().onTrue(start.cmd());
        return routine;
    }

    public static AutoRoutine fourMeterTestSpin() {
        AutoRoutine routine = factory.newRoutine("4 Meter Spin Test");
        AutoTrajectory start = ChoreoTraj.FourMeterSpinTest.asAutoTraj(routine);
        routine.active().onTrue(start.cmd());
        return routine;
    }

    public static AutoRoutine centerPathTest() {
        AutoRoutine routine = factory.newRoutine("Center Path Test");

        AutoTrajectory start = ChoreoTraj.CenterPath$0.asAutoTraj(routine);
        AutoTrajectory back = ChoreoTraj.CenterPath$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                ShootingCommands.autonShoot().withTimeout(5.0),
                start.cmd()
                    .alongWith(RobotContainer.s_Intake.runIntake().withTimeout(5.0)),
                back.cmd(),
                ShootingCommands.autonShoot()));

        return routine;
    }

    public static AutoRoutine rightRebound() {
        AutoRoutine routine = factory.newRoutine("Right Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.RRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToShoot1 = ChoreoTraj.RRebound$1.asAutoTraj(routine);
        AutoTrajectory shootToMidToTrench = ChoreoTraj.RRebound$2.asAutoTraj(routine);
        AutoTrajectory trenchToShoot2 = ChoreoTraj.RRebound$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot1.spawnCmd()
            )
        );

        trenchToShoot1.active().whileTrue(RobotContainer.s_Intake.outtakeIntake());
        trenchToShoot2.active().whileTrue(RobotContainer.s_Intake.outtakeIntake());

        trenchToShoot1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot1),
                ShootingCommands.autonShoot().withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                shootToMidToTrench.spawnCmd()
            )
        );

        shootToMidToTrench.done().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot2.spawnCmd()
            )
        );

        trenchToShoot2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot2),
                ShootingCommands.autonShoot(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public static AutoRoutine leftRebound() {
        AutoRoutine routine = factory.newRoutine("Left Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToShoot1 = ChoreoTraj.LRebound$1.asAutoTraj(routine);
        AutoTrajectory shootToMidToTrench = ChoreoTraj.LRebound$2.asAutoTraj(routine);
        AutoTrajectory trenchToShoot2 = ChoreoTraj.LRebound$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(Intake.State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot1.spawnCmd()
            )
        );

        trenchToShoot1.active().whileTrue(RobotContainer.s_Intake.outtakeIntake());
        trenchToShoot2.active().whileTrue(RobotContainer.s_Intake.outtakeIntake());

        trenchToShoot1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot1),
                ShootingCommands.autonShoot().withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                shootToMidToTrench.spawnCmd()
            )
        );

        shootToMidToTrench.done().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot2.spawnCmd()
            )
        );

        trenchToShoot2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot2),
                ShootingCommands.autonShoot(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public static AutoRoutine leftRebound112() {
        AutoRoutine routine = factory.newRoutine("Left Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToShoot1 = ChoreoTraj.LRebound$1.asAutoTraj(routine);
        AutoTrajectory shootToMidToTrench = ChoreoTraj.LRebound$2.asAutoTraj(routine);
        AutoTrajectory trenchToShoot2 = ChoreoTraj.LRebound$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                ShootingCommands.autonShoot(),
                Commands.waitSeconds(10.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(Intake.State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot1.spawnCmd()
            )
        );

        trenchToShoot1.active().whileTrue(RobotContainer.s_Intake.outtakeIntake());
        trenchToShoot2.active().whileTrue(RobotContainer.s_Intake.outtakeIntake());

        trenchToShoot1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot1),
                ShootingCommands.autonShoot().withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                shootToMidToTrench.spawnCmd()
            )
        );

        shootToMidToTrench.done().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot2.spawnCmd()
            )
        );

        trenchToShoot2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot2),
                ShootingCommands.autonShoot(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public static AutoRoutine rightDoubleRebound() {
        AutoRoutine routine = factory.newRoutine("Right Double Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.RDoubleRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.RDoubleRebound$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToMidToTrench),
                ShootingCommands.autonShoot().withTimeout(5.0),
//                ShootingCommands.autonShoot().until(s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToReboundToTrench),
                ShootingCommands.autonShoot().withTimeout(5.0),
//                ShootingCommands.autonShoot().until(s_Indexer::emptyFuel),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public static AutoRoutine leftDoubleRebound() {
        AutoRoutine routine = factory.newRoutine("Left Double Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LDoubleRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.LDoubleRebound$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToMidToTrench),
                ShootingCommands.autonShoot().withTimeout(5.0),
        //                ShootingCommands.autonShoot().until(s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToReboundToTrench),
                ShootingCommands.autonShoot().withTimeout(5.0),
        //                ShootingCommands.autonShoot().until(s_Indexer::emptyFuel),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public static AutoRoutine rightOP() {
        AutoRoutine routine = factory.newRoutine("Right OP Auton");

        AutoTrajectory trenchPickUpCrossBump1 = ChoreoTraj.ROPAuton$0.asAutoTraj(routine);
        AutoTrajectory shootingSection1 = ChoreoTraj.ROPAuton$1.asAutoTraj(routine);
        AutoTrajectory trenchPickUpCrossBump2 = ChoreoTraj.ROPAuton$2.asAutoTraj(routine);
        AutoTrajectory shootingSection2 = ChoreoTraj.ROPAuton$3.asAutoTraj(routine);
        AutoTrajectory finalRebound = ChoreoTraj.ROPAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchPickUpCrossBump1.resetOdometry(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                trenchPickUpCrossBump1.spawnCmd()
            )
        );

        trenchPickUpCrossBump1.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchPickUpCrossBump2.active().whileTrue(RobotContainer.s_Intake.runIntake());
        finalRebound.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchPickUpCrossBump1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump1),
                shootingSection1.spawnCmd()
            )
        );

        shootingSection1.active().whileTrue(ShootingCommands.autonShoot());
        shootingSection2.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection1),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchPickUpCrossBump2.spawnCmd()
            )
        );

        trenchPickUpCrossBump2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump2),
                shootingSection2.spawnCmd()
            )
        );

        shootingSection2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection2),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                finalRebound.spawnCmd()
            )
        );

        return routine;
    }

    public static AutoRoutine leftOP() {
        AutoRoutine routine = factory.newRoutine("Left OP Auton");

        AutoTrajectory trenchPickUpCrossBump1 = ChoreoTraj.LOPAuton$0.asAutoTraj(routine);
        AutoTrajectory shootingSection1 = ChoreoTraj.LOPAuton$1.asAutoTraj(routine);
        AutoTrajectory trenchPickUpCrossBump2 = ChoreoTraj.LOPAuton$2.asAutoTraj(routine);
        AutoTrajectory shootingSection2 = ChoreoTraj.LOPAuton$3.asAutoTraj(routine);
        AutoTrajectory finalRebound = ChoreoTraj.LOPAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchPickUpCrossBump1.resetOdometry(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                trenchPickUpCrossBump1.spawnCmd()
            )
        );

        trenchPickUpCrossBump1.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchPickUpCrossBump2.active().whileTrue(RobotContainer.s_Intake.runIntake());
        finalRebound.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchPickUpCrossBump1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump1),
                shootingSection1.spawnCmd()
            )
        );

        shootingSection1.active().whileTrue(ShootingCommands.autonShoot());
        shootingSection2.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection1),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchPickUpCrossBump2.spawnCmd()
            )
        );

        trenchPickUpCrossBump2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump2),
                shootingSection2.spawnCmd()
            )
        );

        shootingSection2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection2),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                finalRebound.spawnCmd()
            )
        );

        return routine;
    }

    public static AutoRoutine rightNotSoOP() {
        AutoRoutine routine = factory.newRoutine("Right Not So OP Auton");

        AutoTrajectory firstPass = ChoreoTraj.RNotSoOPAuton$0.asAutoTraj(routine);
        AutoTrajectory secondPass = ChoreoTraj.RNotSoOPAuton$1.asAutoTraj(routine);
        AutoTrajectory thirdPass = ChoreoTraj.RNotSoOPAuton$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                firstPass.resetOdometry(),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                firstPass.spawnCmd()
            )
        );

        firstPass.active().whileTrue(RobotContainer.s_Intake.runIntake());
        secondPass.active().whileTrue(RobotContainer.s_Intake.runIntake());
        thirdPass.active().whileTrue(RobotContainer.s_Intake.runIntake());

        firstPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(firstPass),
                ShootingCommands.autonShoot().withTimeout(2.0),
                ShootingCommands.autonShoot().until(RobotContainer.s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                secondPass.spawnCmd()
            )
        );

        secondPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(secondPass),
                ShootingCommands.autonShoot().withTimeout(2.0),
                ShootingCommands.autonShoot().until(RobotContainer.s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                thirdPass.spawnCmd()
            )
        );

        return routine;
    }

    public static AutoRoutine leftNotSoOP() {
        AutoRoutine routine = factory.newRoutine("Left Not So OP Auton");

        AutoTrajectory firstPass = ChoreoTraj.LNotSoOPAuton$0.asAutoTraj(routine);
        AutoTrajectory secondPass = ChoreoTraj.LNotSoOPAuton$1.asAutoTraj(routine);
        AutoTrajectory thirdPass = ChoreoTraj.LNotSoOPAuton$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                firstPass.resetOdometry(),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                firstPass.spawnCmd()
            )
        );

        firstPass.active().whileTrue(RobotContainer.s_Intake.runIntake());
        secondPass.active().whileTrue(RobotContainer.s_Intake.runIntake());
        thirdPass.active().whileTrue(RobotContainer.s_Intake.runIntake());

        firstPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(firstPass),
                ShootingCommands.autonShoot().withTimeout(2.0),
                ShootingCommands.autonShoot().until(RobotContainer.s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                secondPass.spawnCmd()
            )
        );

        secondPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(secondPass),
                ShootingCommands.autonShoot().withTimeout(2.0),
                ShootingCommands.autonShoot().until(RobotContainer.s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                thirdPass.spawnCmd()
            )
        );

        return routine;
    }

    public static AutoRoutine middleDepotAuton() {
        AutoRoutine routine = factory.newRoutine("Middle Depot Auton");

        AutoTrajectory moveToShootPose = ChoreoTraj.MiddleDepotAuton$0.asAutoTraj(routine);
        AutoTrajectory shootToDepotToShoot = ChoreoTraj.MiddleDepotAuton$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                moveToShootPose.resetOdometry(),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                moveToShootPose.spawnCmd()
            )
        );

        moveToShootPose.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToDepotToShoot.active().whileTrue(RobotContainer.s_Intake.runIntake());

        moveToShootPose.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(moveToShootPose),
                ShootingCommands.autonShoot().withTimeout(2.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        shootToDepotToShoot.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootToDepotToShoot),
                ShootingCommands.autonShoot().withTimeout(3.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        return routine;
    }

    public static AutoRoutine middleDepotAuton2() {
        AutoRoutine routine = factory.newRoutine("Middle Depot Auton 2");

        AutoTrajectory moveToShootPose = ChoreoTraj.MiddleDepotAuton2$0.asAutoTraj(routine);
        AutoTrajectory shootToDepotToShoot = ChoreoTraj.MiddleDepotAuton2$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                moveToShootPose.resetOdometry(),
                RobotContainer.s_Intake.setDesiredStateCommand(State.INTAKE),
                moveToShootPose.spawnCmd()
            )
        );

        moveToShootPose.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToDepotToShoot.active().whileTrue(RobotContainer.s_Intake.runIntake());

        moveToShootPose.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(moveToShootPose),
                ShootingCommands.autonShoot().withTimeout(2.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        shootToDepotToShoot.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootToDepotToShoot),
                ShootingCommands.autonShoot().withTimeout(5.0)
            )
        );

        return routine;
    }

    public static AutoRoutine rightStuy() {
        AutoRoutine routine = factory.newRoutine("Right Stuy Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.RStuyAuton$0.asAutoTraj(routine);
        AutoTrajectory collectFromNeutral = ChoreoTraj.RStuyAuton$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.RStuyAuton$2.asAutoTraj(routine);
        AutoTrajectory towerAlign = ChoreoTraj.RStuyAuton$3.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.RStuyAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(ShootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                collectFromNeutral.spawnCmd()
            )
        );

        collectFromNeutral.active().whileTrue(s_Intake.runIntake());

        collectFromNeutral.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromNeutral),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                towerAlign.spawnCmd()
            )
        );

        towerAlign.active().whileTrue(s_Intake.runIntake());

        towerAlign.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(towerAlign),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(ShootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(ShootingCommands.autonShoot());

        return routine;
    }

    public static AutoRoutine leftStuy() {
        AutoRoutine routine = factory.newRoutine("Left Stuy Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.LStuyAuton$0.asAutoTraj(routine);
        AutoTrajectory collectFromNeutral = ChoreoTraj.LStuyAuton$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.LStuyAuton$2.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.LStuyAuton$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(ShootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                collectFromNeutral.spawnCmd()
            )
        );

        collectFromNeutral.active().whileTrue(s_Intake.runIntake());

        collectFromNeutral.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromNeutral),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(ShootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(ShootingCommands.autonShoot());

        return routine;
    }

    public static AutoRoutine rightBumpHubDepot() {
        AutoRoutine routine = factory.newRoutine("Right Bump Hub Depot Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.RBumpHubDepot$0.asAutoTraj(routine);
        AutoTrajectory collectFromBackHub = ChoreoTraj.RBumpHubDepot$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.RBumpHubDepot$2.asAutoTraj(routine);
        AutoTrajectory towerAlign = ChoreoTraj.RBumpHubDepot$3.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.RBumpHubDepot$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(ShootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                collectFromBackHub.spawnCmd()
            )
        );

        collectFromBackHub.active().whileTrue(s_Intake.runIntake());

        collectFromBackHub.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromBackHub),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                towerAlign.spawnCmd()
            )
        );

        towerAlign.active().whileTrue(s_Intake.setDesiredStateCommand(State.INTAKE));

        towerAlign.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(towerAlign),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(ShootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(ShootingCommands.autonShoot());

        return routine;
    }

    public static AutoRoutine leftBumpHubDepot() {
        AutoRoutine routine = factory.newRoutine("Left Bump Hub Depot Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.LBumpHubDepot$0.asAutoTraj(routine);
        AutoTrajectory collectFromBackHub = ChoreoTraj.LBumpHubDepot$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.LBumpHubDepot$2.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.LBumpHubDepot$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(ShootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                s_Intake.setDesiredStateCommand(State.INTAKE),
                collectFromBackHub.spawnCmd()
            )
        );

        collectFromBackHub.active().whileTrue(s_Intake.runIntake());

        collectFromBackHub.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromBackHub),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(ShootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(ShootingCommands.autonShoot());

        return routine;
    }
}
