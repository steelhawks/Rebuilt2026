package org.steelhawks;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
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
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;

@SuppressWarnings("unused")
public final class Autos {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Intake s_Intake = RobotContainer.s_Intake;
    private static final Indexer s_Indexer = RobotContainer.s_Indexer;

    private static final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser");

    private static final Choreo.TrajectoryLogger<SwerveSample> trajLogger = (trajectory, starting) -> {
        Pose2d[] poses = Arrays.stream(trajectory.getPoses())
            .map(AllianceFlip::apply)
            .toArray(Pose2d[]::new);
        FieldConstants.FIELD_2D.getObject("Path").setPoses(List.of(poses));
        Logger.recordOutput("Odometry/Trajectory", poses);
    };

    private static final AutoFactory factory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose,
            s_Swerve::setPose,
            (SwerveSample sample) -> s_Swerve.followTrajectory(sample, false),
            true,
            s_Swerve,
            trajLogger
        );

    private static final AutoFactory flippedFactory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose,
            s_Swerve::setPose,
            (SwerveSample sample) -> s_Swerve.followTrajectory(sample, true),
            true,
            s_Swerve,
            trajLogger
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
    public static double replanDistanceRequirement = Units.inchesToMeters(15.0); // tune
    private static boolean tuningOptionsAdded = false;

    /**
     * After a trajectory finishes, checks if the robot was bumped away from the
     * trajectory's end pose and recovers accordingly:
     * Call this AFTER a trajectory ends, BEFORE shooting or spawning the next trajectory.
     */
    public static Command recoverToTrajectoryEnd(AutoTrajectory traj, boolean isLeft) {
        return Commands.defer(() -> {
            Optional<Pose2d> maybeFinalPose = traj.getFinalPose();

            Pose2d finalPose;
            if (maybeFinalPose.isPresent()) {
                Pose2d rawFinal = maybeFinalPose.get();
                finalPose = isLeft
                    ? new Pose2d(
                        rawFinal.getX(),
                        FieldConstants.FIELD_WIDTH - rawFinal.getY(),
                        new Rotation2d(-rawFinal.getRotation().getRadians()))
                    : rawFinal;
            } else {
                finalPose = RobotState.getInstance().getEstimatedPose();
            }

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
        autoChooser.addOption("4 Meter Test", fourMeterTest().cmd().withName(ChoreoTraj.FourMeterTest.name()));
        autoChooser.addOption("4 Meter Spin Test", fourMeterTestSpin().cmd().withName(ChoreoTraj.FourMeterSpinTest.name()));
        autoChooser.addOption("Center Path Test", centerPathTest().cmd().withName(ChoreoTraj.CenterPath.name()));
        autoChooser.addOption("Right Rebound Auton", rightRebound().cmd().withName(ChoreoTraj.RRebound.name()));
//        autoChooser.addOption("Right Double Rebound Auton", rightDoubleRebound().cmd().withName(ChoreoTraj.RDoubleRebound.name()));
        autoChooser.addOption("Left Rebound Auton", leftRebound().cmd().withName(ChoreoTraj.LRebound.name()));
        autoChooser.addOption("Right OP Auton", rightOP().cmd().withName(ChoreoTraj.OPAuton.name()));
        autoChooser.addOption("Right Not So OP Auton", rightNotSoOP().cmd().withName(ChoreoTraj.NotSoOPAuton.name()));
        autoChooser.addOption("Middle Depot Auton", middleDepotAuton().cmd().withName(ChoreoTraj.MiddleDepotAuton.name()));
        autoChooser.addOption("Middle Depot Auton 2", middleDepotAuton2().cmd().withName(ChoreoTraj.MiddleDepotAuton2.name()));

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

    public static AutoRoutine rebound(boolean isLeft) {
        AutoRoutine routine = (isLeft ? flippedFactory : factory).newRoutine((isLeft ? "Left" : "Right") + " Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.RRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.RRebound$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                Commands.defer(() -> {
                    if (isLeft) {
                        Pose2d bluePose = trenchToMidToTrench.getInitialPose()
                            .orElse(Pose2d.kZero);
                        Pose2d flipped = new Pose2d(
                            bluePose.getX(),
                            FieldConstants.FIELD_WIDTH - bluePose.getY(),
                            new Rotation2d(-bluePose.getRotation().getRadians()));
                        return Commands.runOnce(() -> s_Swerve.setPose(AllianceFlip.apply(flipped)));
                    } else {
                        return trenchToMidToTrench.resetOdometry();
                    }
                }, Set.of()),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToMidToTrench, isLeft),
                ShootingCommands.shoot().withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToReboundToTrench, isLeft),
                ShootingCommands.shoot(),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public static AutoRoutine leftRebound() {
        return rebound(true);
    }

    public static AutoRoutine rightRebound() {
        return rebound(false);
    }

    public static AutoRoutine rightOP() {
        AutoRoutine routine = factory.newRoutine("Right OP Auton");

        AutoTrajectory trenchPickUpCrossBump1 = ChoreoTraj.OPAuton$0.asAutoTraj(routine);
        AutoTrajectory shootingSection1 = ChoreoTraj.OPAuton$1.asAutoTraj(routine);
        AutoTrajectory trenchPickUpCrossBump2 = ChoreoTraj.OPAuton$2.asAutoTraj(routine);
        AutoTrajectory shootingSection2 = ChoreoTraj.OPAuton$3.asAutoTraj(routine);
        AutoTrajectory finalRebound = ChoreoTraj.OPAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
                trenchPickUpCrossBump1.spawnCmd()
            )
        );

        trenchPickUpCrossBump1.active().whileTrue(RobotContainer.s_Intake.runIntake());
        trenchPickUpCrossBump2.active().whileTrue(RobotContainer.s_Intake.runIntake());
        finalRebound.active().whileTrue(RobotContainer.s_Intake.runIntake());

        trenchPickUpCrossBump1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump1, false),
                shootingSection1.spawnCmd()
            )
        );

        shootingSection1.active().whileTrue(ShootingCommands.autonShoot());
        shootingSection2.active().whileTrue(ShootingCommands.autonShoot());

        shootingSection1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection1, false),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchPickUpCrossBump2.spawnCmd()
            )
        );

        trenchPickUpCrossBump2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump2, false),
                shootingSection2.spawnCmd()
            )
        );

        shootingSection2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection2, false),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                finalRebound.spawnCmd()
            )
        );

        return routine;
    }

    public static AutoRoutine rightNotSoOP() {
        AutoRoutine routine = factory.newRoutine("Right Not So OP Auton");

        AutoTrajectory firstPass = ChoreoTraj.NotSoOPAuton$0.asAutoTraj(routine);
        AutoTrajectory secondPass = ChoreoTraj.NotSoOPAuton$1.asAutoTraj(routine);
        AutoTrajectory thirdPass = ChoreoTraj.NotSoOPAuton$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
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
                recoverToTrajectoryEnd(firstPass, false),
                ShootingCommands.autonShoot().withTimeout(2.0),
                ShootingCommands.autonShoot().until(RobotContainer.s_Indexer::emptyFuel).withTimeout(5.0),
                RobotContainer.s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                secondPass.spawnCmd()
            )
        );

        secondPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(secondPass, false),
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
                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
                moveToShootPose.spawnCmd()
            )
        );

        moveToShootPose.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToDepotToShoot.active().whileTrue(RobotContainer.s_Intake.runIntake());

        moveToShootPose.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(moveToShootPose, false),
                ShootingCommands.autonShoot().withTimeout(2.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        shootToDepotToShoot.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootToDepotToShoot, false),
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
                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE),
                moveToShootPose.spawnCmd()
            )
        );

        moveToShootPose.active().whileTrue(RobotContainer.s_Intake.runIntake());
        shootToDepotToShoot.active().whileTrue(RobotContainer.s_Intake.runIntake());

        moveToShootPose.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(moveToShootPose, false),
                ShootingCommands.autonShoot().withTimeout(2.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        shootToDepotToShoot.done().onTrue(
            Commands.sequence(
                Commands.runOnce(RobotContainer.s_Swerve::stopWithX),
                recoverToTrajectoryEnd(shootToDepotToShoot, false),
                ShootingCommands.autonShoot().withTimeout(5.0)
            )
        );

        return routine;
    }
}
