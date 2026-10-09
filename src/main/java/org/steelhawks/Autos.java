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
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

@SuppressWarnings("unused")
public final class Autos {

    // Captured once at construction. Autos requires the full superstructure;
    // these throw with descriptive errors if invoked on a robot missing one.
    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Turret turret;
    private final ShootingCommands shootingCommands;

    private final LoggedDashboardChooser<Command> autoChooser =
        new LoggedDashboardChooser<>("Auto Chooser");

    private final AutoFactory factory;

    // Distance threshold to decide between pathfinding vs simple PID recovery
    public static double replanDistanceRequirement = Units.inchesToMeters(5.0); // tune
    private boolean tuningOptionsAdded = false;

    public Autos(Subsystems s, ShootingCommands shootingCommands) {
        this.swerve = Subsystems.swerve();
        this.intake = Subsystems.intake();
        this.indexer = Subsystems.indexer();
        this.hood = Subsystems.hood();
        this.flywheel = Subsystems.flywheel();
        this.turret = Subsystems.turret();
        this.shootingCommands = shootingCommands;

        this.factory =
            new AutoFactory(
                RobotState.getInstance()::getEstimatedPose,
                swerve::setPose,
                swerve::followTrajectory,
                true,
                swerve,
                ((trajectory, starting) -> {
                    Pose2d[] poses = Arrays.stream(trajectory.getPoses())
                        .map(AllianceFlip::apply)
                        .toArray(Pose2d[]::new);
                    FieldConstants.FIELD_2D.getObject("Path").setPoses(List.of(poses));
                    Logger.recordOutput("Odometry/Trajectory", poses);
                })
            );
    }

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

    /**
     * After a trajectory finishes, checks if the robot was bumped away from the
     * trajectory's end pose and recovers accordingly:
     * Call this AFTER a trajectory ends, BEFORE shooting or spawning the next trajectory.
     */
    public Command recoverToTrajectoryEnd(AutoTrajectory traj) {
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
        }, Set.of(swerve));
    }

    public void init() {
        autoChooser.addOption("No Auton", Commands.none().withName("No Auton"));
//        autoChooser.addOption("4 Meter Test", fourMeterTest().cmd().withName(ChoreoTraj.FourMeterTest.name()));
//        autoChooser.addOption("4 Meter Spin Test", fourMeterTestSpin().cmd().withName(ChoreoTraj.FourMeterSpinTest.name()));
//        autoChooser.addOption("Center Path Test", centerPathTest().cmd().withName(ChoreoTraj.CenterPath.name()));
        autoChooser.addOption("Right Rebound Auton", rightRebound().cmd().withName(ChoreoTraj.RRebound.name()));
        autoChooser.addOption("Left Rebound Auton", leftRebound().cmd().withName(ChoreoTraj.LRebound.name()));
        autoChooser.addOption("Left Rebound Auton Q112", leftRebound112().cmd().withName(ChoreoTraj.LRebound.name() + "112"));
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
        autoChooser.addOption("Stationary Shoot", shootingCommands.shoot());

        if (Toggles.tuningMode.get()) {
            pollTuningMode();
        }
    }

    public void pollTuningMode() {
        if (!tuningOptionsAdded && Toggles.tuningMode.get()) {

            autoChooser.addOption("Swerve Drive (Quick Characterizer)", DriveCommands.feedforwardCharacterization(swerve));

            autoChooser.addOption("Swerve Drive (Quasistatic Forward)", swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Drive (Quasistatic Backward)", swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Swerve Drive (Dynamic Forward)", swerve.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Drive (Dynamic Backward)", swerve.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Turn (Quasistatic Forward)", swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Turn (Quasistatic Backward)", swerve.turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Swerve Turn (Dynamic Forward)", swerve.turnSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Turn (Dynamic Backward)", swerve.turnSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Swerve Angular (Quasistatic Forward)", swerve.angularSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Angular (Quasistatic Backward)", swerve.angularSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Swerve Angular (Dynamic Forward)", swerve.angularSysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Swerve Angular (Dynamic Backward)", swerve.angularSysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Flywheel (Quick Characterizer)", flywheel.feedforwardCharacterization());
            autoChooser.addOption("Flywheel (Quasistatic Forward)", flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Flywheel (Quasistatic Backward)", flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Flywheel (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Flywheel (Quasistatic Backward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            autoChooser.addOption("Turret (Quick Characterizer)", turret.feedforwardCharacterization());
            autoChooser.addOption("Hood (Quick Characterizer)", hood.feedforwardCharacterization());
            tuningOptionsAdded = true;
        }
    }

    public Misalignment getMisalignment() {
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

    public Command followTrajectory(String pathPlanner) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathPlanner);
            return DriveCommands.followPath(path).withName("Following " + pathPlanner);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public Command getAuto() {
        return autoChooser.get();
    }

    public AutoRoutine fourMeterTest() {
        AutoRoutine routine = factory.newRoutine("4 Meter Test");
        AutoTrajectory start = ChoreoTraj.FourMeterTest.asAutoTraj(routine);
        routine.active().onTrue(start.cmd());
        return routine;
    }

    public AutoRoutine fourMeterTestSpin() {
        AutoRoutine routine = factory.newRoutine("4 Meter Spin Test");
        AutoTrajectory start = ChoreoTraj.FourMeterSpinTest.asAutoTraj(routine);
        routine.active().onTrue(start.cmd());
        return routine;
    }

    public AutoRoutine centerPathTest() {
        AutoRoutine routine = factory.newRoutine("Center Path Test");

        AutoTrajectory start = ChoreoTraj.CenterPath$0.asAutoTraj(routine);
        AutoTrajectory back = ChoreoTraj.CenterPath$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootingCommands.autonShoot().withTimeout(5.0),
                start.cmd()
                    .alongWith(intake.runIntake().withTimeout(5.0)),
                back.cmd(),
                shootingCommands.autonShoot()));

        return routine;
    }

    public AutoRoutine rightRebound() {
        AutoRoutine routine = factory.newRoutine("Right Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.RRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToShoot1 = ChoreoTraj.RRebound$1.asAutoTraj(routine);
        AutoTrajectory shootToMidToTrench = ChoreoTraj.RRebound$2.asAutoTraj(routine);
        AutoTrajectory trenchToShoot2 = ChoreoTraj.RRebound$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(intake.runIntake());
        shootToMidToTrench.active().whileTrue(intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot1.spawnCmd()
            )
        );

        trenchToShoot1.active().whileTrue(intake.outtakeIntake());
        trenchToShoot2.active().whileTrue(intake.outtakeIntake());

        trenchToShoot1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot1),
                shootingCommands.autonShoot().withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                shootToMidToTrench.spawnCmd()
            )
        );

        shootToMidToTrench.done().onTrue(
            Commands.sequence(
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot2.spawnCmd()
            )
        );

        trenchToShoot2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot2),
                shootingCommands.autonShoot(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public AutoRoutine leftRebound() {
        AutoRoutine routine = factory.newRoutine("Left Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToShoot1 = ChoreoTraj.LRebound$1.asAutoTraj(routine);
        AutoTrajectory shootToMidToTrench = ChoreoTraj.LRebound$2.asAutoTraj(routine);
        AutoTrajectory trenchToShoot2 = ChoreoTraj.LRebound$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(Intake.State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(intake.runIntake());
        shootToMidToTrench.active().whileTrue(intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot1.spawnCmd()
            )
        );

        trenchToShoot1.active().whileTrue(intake.outtakeIntake());
        trenchToShoot2.active().whileTrue(intake.outtakeIntake());

        trenchToShoot1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot1),
                shootingCommands.autonShoot().withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                shootToMidToTrench.spawnCmd()
            )
        );

        shootToMidToTrench.done().onTrue(
            Commands.sequence(
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot2.spawnCmd()
            )
        );

        trenchToShoot2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot2),
                shootingCommands.autonShoot(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public AutoRoutine leftRebound112() {
        AutoRoutine routine = factory.newRoutine("Left Rebound Auton 112");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToShoot1 = ChoreoTraj.LRebound$1.asAutoTraj(routine);
        AutoTrajectory shootToMidToTrench = ChoreoTraj.LRebound$2.asAutoTraj(routine);
        AutoTrajectory trenchToShoot2 = ChoreoTraj.LRebound$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                shootingCommands.shoot().withTimeout(10.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(Intake.State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(intake.runIntake());
        shootToMidToTrench.active().whileTrue(intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot1.spawnCmd()
            )
        );

        trenchToShoot1.active().whileTrue(intake.outtakeIntake());
        trenchToShoot2.active().whileTrue(intake.outtakeIntake());

        trenchToShoot1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot1),
                shootingCommands.autonShoot().withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                shootToMidToTrench.spawnCmd()
            )
        );

        shootToMidToTrench.done().onTrue(
            Commands.sequence(
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToShoot2.spawnCmd()
            )
        );

        trenchToShoot2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToShoot2),
                shootingCommands.autonShoot(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public AutoRoutine rightDoubleRebound() {
        AutoRoutine routine = factory.newRoutine("Right Double Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.RDoubleRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.RDoubleRebound$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToMidToTrench),
                shootingCommands.autonShoot().withTimeout(5.0),
//                shootingCommands.autonShoot().until(indexer::emptyFuel).withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToReboundToTrench),
                shootingCommands.autonShoot().withTimeout(5.0),
//                shootingCommands.autonShoot().until(indexer::emptyFuel),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public AutoRoutine leftDoubleRebound() {
        AutoRoutine routine = factory.newRoutine("Left Double Rebound Auton");

        AutoTrajectory trenchToMidToTrench = ChoreoTraj.LDoubleRebound$0.asAutoTraj(routine);
        AutoTrajectory trenchToReboundToTrench = ChoreoTraj.LDoubleRebound$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchToMidToTrench.resetOdometry(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(State.INTAKE),
                trenchToMidToTrench.spawnCmd()
            )
        );

        trenchToMidToTrench.active().whileTrue(intake.runIntake());
        trenchToReboundToTrench.active().whileTrue(intake.runIntake());

        trenchToMidToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToMidToTrench),
                shootingCommands.autonShoot().withTimeout(5.0),
        //                shootingCommands.autonShoot().until(indexer::emptyFuel).withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchToReboundToTrench.spawnCmd()
            )
        );

        trenchToReboundToTrench.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchToReboundToTrench),
                shootingCommands.autonShoot().withTimeout(5.0),
        //                shootingCommands.autonShoot().until(indexer::emptyFuel),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0))
            )
        );

        return routine;
    }

    public AutoRoutine rightOP() {
        AutoRoutine routine = factory.newRoutine("Right OP Auton");

        AutoTrajectory trenchPickUpCrossBump1 = ChoreoTraj.ROPAuton$0.asAutoTraj(routine);
        AutoTrajectory shootingSection1 = ChoreoTraj.ROPAuton$1.asAutoTraj(routine);
        AutoTrajectory trenchPickUpCrossBump2 = ChoreoTraj.ROPAuton$2.asAutoTraj(routine);
        AutoTrajectory shootingSection2 = ChoreoTraj.ROPAuton$3.asAutoTraj(routine);
        AutoTrajectory finalRebound = ChoreoTraj.ROPAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchPickUpCrossBump1.resetOdometry(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(State.INTAKE),
                trenchPickUpCrossBump1.spawnCmd()
            )
        );

        trenchPickUpCrossBump1.active().whileTrue(intake.runIntake());
        trenchPickUpCrossBump2.active().whileTrue(intake.runIntake());
        finalRebound.active().whileTrue(intake.runIntake());

        trenchPickUpCrossBump1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump1),
                shootingSection1.spawnCmd()
            )
        );

        shootingSection1.active().whileTrue(shootingCommands.autonShoot());
        shootingSection2.active().whileTrue(shootingCommands.autonShoot());

        shootingSection1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection1),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchPickUpCrossBump2.spawnCmd()
            )
        );

        trenchPickUpCrossBump2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump2),
                shootingSection2.spawnCmd()
            )
        );

        shootingSection2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection2),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                finalRebound.spawnCmd()
            )
        );

        return routine;
    }

    public AutoRoutine leftOP() {
        AutoRoutine routine = factory.newRoutine("Left OP Auton");

        AutoTrajectory trenchPickUpCrossBump1 = ChoreoTraj.LOPAuton$0.asAutoTraj(routine);
        AutoTrajectory shootingSection1 = ChoreoTraj.LOPAuton$1.asAutoTraj(routine);
        AutoTrajectory trenchPickUpCrossBump2 = ChoreoTraj.LOPAuton$2.asAutoTraj(routine);
        AutoTrajectory shootingSection2 = ChoreoTraj.LOPAuton$3.asAutoTraj(routine);
        AutoTrajectory finalRebound = ChoreoTraj.LOPAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                trenchPickUpCrossBump1.resetOdometry(),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                intake.setDesiredStateCommand(State.INTAKE),
                trenchPickUpCrossBump1.spawnCmd()
            )
        );

        trenchPickUpCrossBump1.active().whileTrue(intake.runIntake());
        trenchPickUpCrossBump2.active().whileTrue(intake.runIntake());
        finalRebound.active().whileTrue(intake.runIntake());

        trenchPickUpCrossBump1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump1),
                shootingSection1.spawnCmd()
            )
        );

        shootingSection1.active().whileTrue(shootingCommands.autonShoot());
        shootingSection2.active().whileTrue(shootingCommands.autonShoot());

        shootingSection1.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection1),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                trenchPickUpCrossBump2.spawnCmd()
            )
        );

        trenchPickUpCrossBump2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(trenchPickUpCrossBump2),
                shootingSection2.spawnCmd()
            )
        );

        shootingSection2.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection2),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                finalRebound.spawnCmd()
            )
        );

        return routine;
    }

    public AutoRoutine rightNotSoOP() {
        AutoRoutine routine = factory.newRoutine("Right Not So OP Auton");

        AutoTrajectory firstPass = ChoreoTraj.RNotSoOPAuton$0.asAutoTraj(routine);
        AutoTrajectory secondPass = ChoreoTraj.RNotSoOPAuton$1.asAutoTraj(routine);
        AutoTrajectory thirdPass = ChoreoTraj.RNotSoOPAuton$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                firstPass.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                firstPass.spawnCmd()
            )
        );

        firstPass.active().whileTrue(intake.runIntake());
        secondPass.active().whileTrue(intake.runIntake());
        thirdPass.active().whileTrue(intake.runIntake());

        firstPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(firstPass),
                shootingCommands.autonShoot().withTimeout(2.0),
                shootingCommands.autonShoot().until(indexer::emptyFuel).withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                secondPass.spawnCmd()
            )
        );

        secondPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(secondPass),
                shootingCommands.autonShoot().withTimeout(2.0),
                shootingCommands.autonShoot().until(indexer::emptyFuel).withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                thirdPass.spawnCmd()
            )
        );

        return routine;
    }

    public AutoRoutine leftNotSoOP() {
        AutoRoutine routine = factory.newRoutine("Left Not So OP Auton");

        AutoTrajectory firstPass = ChoreoTraj.LNotSoOPAuton$0.asAutoTraj(routine);
        AutoTrajectory secondPass = ChoreoTraj.LNotSoOPAuton$1.asAutoTraj(routine);
        AutoTrajectory thirdPass = ChoreoTraj.LNotSoOPAuton$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                firstPass.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                firstPass.spawnCmd()
            )
        );

        firstPass.active().whileTrue(intake.runIntake());
        secondPass.active().whileTrue(intake.runIntake());
        thirdPass.active().whileTrue(intake.runIntake());

        firstPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(firstPass),
                shootingCommands.autonShoot().withTimeout(2.0),
                shootingCommands.autonShoot().until(indexer::emptyFuel).withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                secondPass.spawnCmd()
            )
        );

        secondPass.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(secondPass),
                shootingCommands.autonShoot().withTimeout(2.0),
                shootingCommands.autonShoot().until(indexer::emptyFuel).withTimeout(5.0),
                hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)),
                thirdPass.spawnCmd()
            )
        );

        return routine;
    }

    public AutoRoutine middleDepotAuton() {
        AutoRoutine routine = factory.newRoutine("Middle Depot Auton");

        AutoTrajectory moveToShootPose = ChoreoTraj.MiddleDepotAuton$0.asAutoTraj(routine);
        AutoTrajectory shootToDepotToShoot = ChoreoTraj.MiddleDepotAuton$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                moveToShootPose.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                moveToShootPose.spawnCmd()
            )
        );

        moveToShootPose.active().whileTrue(intake.runIntake());
        shootToDepotToShoot.active().whileTrue(intake.runIntake());

        moveToShootPose.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(moveToShootPose),
                shootingCommands.autonShoot().withTimeout(2.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        shootToDepotToShoot.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootToDepotToShoot),
                shootingCommands.autonShoot().withTimeout(3.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        return routine;
    }

    public AutoRoutine middleDepotAuton2() {
        AutoRoutine routine = factory.newRoutine("Middle Depot Auton 2");

        AutoTrajectory moveToShootPose = ChoreoTraj.MiddleDepotAuton2$0.asAutoTraj(routine);
        AutoTrajectory shootToDepotToShoot = ChoreoTraj.MiddleDepotAuton2$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                moveToShootPose.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                moveToShootPose.spawnCmd()
            )
        );

        moveToShootPose.active().whileTrue(intake.runIntake());
        shootToDepotToShoot.active().whileTrue(intake.runIntake());

        moveToShootPose.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(moveToShootPose),
                shootingCommands.autonShoot().withTimeout(2.0),
                shootToDepotToShoot.spawnCmd()
            )
        );

        shootToDepotToShoot.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootToDepotToShoot),
                shootingCommands.autonShoot().withTimeout(5.0)
            )
        );

        return routine;
    }

    public AutoRoutine rightStuy() {
        AutoRoutine routine = factory.newRoutine("Right Stuy Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.RStuyAuton$0.asAutoTraj(routine);
        AutoTrajectory collectFromNeutral = ChoreoTraj.RStuyAuton$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.RStuyAuton$2.asAutoTraj(routine);
        AutoTrajectory towerAlign = ChoreoTraj.RStuyAuton$3.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.RStuyAuton$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(shootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                intake.setDesiredStateCommand(State.INTAKE),
                collectFromNeutral.spawnCmd()
            )
        );

        collectFromNeutral.active().whileTrue(intake.runIntake());

        collectFromNeutral.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromNeutral),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(shootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                intake.setDesiredStateCommand(State.INTAKE),
                towerAlign.spawnCmd()
            )
        );

        towerAlign.active().whileTrue(intake.runIntake());

        towerAlign.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(towerAlign),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(shootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(shootingCommands.autonShoot());

        return routine;
    }

    public AutoRoutine leftStuy() {
        AutoRoutine routine = factory.newRoutine("Left Stuy Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.LStuyAuton$0.asAutoTraj(routine);
        AutoTrajectory collectFromNeutral = ChoreoTraj.LStuyAuton$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.LStuyAuton$2.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.LStuyAuton$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(shootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                intake.setDesiredStateCommand(State.INTAKE),
                collectFromNeutral.spawnCmd()
            )
        );

        collectFromNeutral.active().whileTrue(intake.runIntake());

        collectFromNeutral.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromNeutral),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(shootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(shootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(shootingCommands.autonShoot());

        return routine;
    }

    public AutoRoutine rightBumpHubDepot() {
        AutoRoutine routine = factory.newRoutine("Right Bump Hub Depot Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.RBumpHubDepot$0.asAutoTraj(routine);
        AutoTrajectory collectFromBackHub = ChoreoTraj.RBumpHubDepot$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.RBumpHubDepot$2.asAutoTraj(routine);
        AutoTrajectory towerAlign = ChoreoTraj.RBumpHubDepot$3.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.RBumpHubDepot$4.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(shootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                intake.setDesiredStateCommand(State.INTAKE),
                collectFromBackHub.spawnCmd()
            )
        );

        collectFromBackHub.active().whileTrue(intake.runIntake());

        collectFromBackHub.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromBackHub),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(shootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                towerAlign.spawnCmd()
            )
        );

        towerAlign.active().whileTrue(intake.setDesiredStateCommand(State.INTAKE));

        towerAlign.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(towerAlign),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(shootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(shootingCommands.autonShoot());

        return routine;
    }

    public AutoRoutine leftBumpHubDepot() {
        AutoRoutine routine = factory.newRoutine("Left Bump Hub Depot Auton");

        AutoTrajectory shootPreloaded = ChoreoTraj.LBumpHubDepot$0.asAutoTraj(routine);
        AutoTrajectory collectFromBackHub = ChoreoTraj.LBumpHubDepot$1.asAutoTraj(routine);
        AutoTrajectory shootingSection = ChoreoTraj.LBumpHubDepot$2.asAutoTraj(routine);
        AutoTrajectory shootIntoDepot = ChoreoTraj.LBumpHubDepot$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                shootPreloaded.resetOdometry(),
                intake.setDesiredStateCommand(State.INTAKE),
                shootPreloaded.spawnCmd()
            )
        );

        shootPreloaded.active().whileTrue(shootingCommands.autonShoot());

        shootPreloaded.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootPreloaded),
                intake.setDesiredStateCommand(State.INTAKE),
                collectFromBackHub.spawnCmd()
            )
        );

        collectFromBackHub.active().whileTrue(intake.runIntake());

        collectFromBackHub.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(collectFromBackHub),
                shootingSection.spawnCmd()
            )
        );

        shootingSection.active().whileTrue(shootingCommands.autonShoot());

        shootingSection.done().onTrue(
            Commands.sequence(
                Commands.runOnce(swerve::stopWithX),
                recoverToTrajectoryEnd(shootingSection),
                shootIntoDepot.spawnCmd()
            )
        );

        shootIntoDepot.active().whileTrue(shootingCommands.shootWhileIntaking());

        shootIntoDepot.done().onTrue(shootingCommands.autonShoot());

        return routine;
    }
}
