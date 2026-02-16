package org.steelhawks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.oldintake.OldIntake;
import org.steelhawks.subsystems.led.LEDMatrix;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.oldintake.OldIntakeConstants;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOTalonFX;
import org.steelhawks.subsystems.superstructure.pivot.Pivot;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;

public class RobotContainer {

    private final RobotConfig config = RobotConfig.getConfig();

    public static LEDMatrix s_LEDMatrix = null;
    public static LEDStrip s_LEDStrip = null;
    public static Swerve s_Swerve = null;
    public static Vision s_Vision = null;
    public static ObjectVision s_ObjVision = null;
    public static Flywheel s_Flywheel = null;
    public static Turret s_Turret = null;
    public static Pivot s_Pivot = null;
    public static OldIntake s_OldIntake = null;
    public static Indexer s_Indexer = null;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);

        s_Swerve = config.createSwerve();
        s_LEDMatrix = config.createLEDMatrix().orElse(null);
        s_LEDStrip = config.createLEDStrip().orElse(null);
        s_Vision = config.createVision().orElse(null);
        s_ObjVision = config.createObjectVision().orElse(null);
//        s_Flywheel = config.createFlywheel().orElse(null);
        s_Turret = config.createTurret(RobotState.getInstance()::getEstimatedPose).orElse(null);
//        s_Pivot = config.createPivot().orElse(null);
//        s_Intake = config.createIntake().orElse(null);
        s_Flywheel = new Flywheel(new FlywheelIOTalonFX(new RobotConfig.CANBus("")));
        s_Indexer = config.createIndexer().orElse(null);
        s_OldIntake = config.createOldIntake().orElse(null);

        if (config.hasAutos) {
            Autos.init();
        }
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));
        driver.x().onTrue(s_Swerve.zeroHeading());
        configureTriggers();
        configureDriver();

        if (config.hasIntake) {
            driver.rightTrigger().whileTrue(s_OldIntake.runIntake());

            driver.x().onTrue(
                s_OldIntake.setDesiredStateCommand(OldIntakeConstants.State.HOME));
            driver.y().onTrue(
                s_OldIntake.setDesiredStateCommand(OldIntakeConstants.State.INTAKE));
        }

//        if (config.hasTurret) {
//            driver.povLeft()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromRadians(Math.PI / 2.0)));
//            driver.povUp()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromRadians(0.0)));
//            driver.povDown()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromRadians(Math.PI)));
//            driver.povRight()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromRadians(-Math.PI / 2.0)));
//        }

        if (config.hasFlywheel) {
//            driver.x()
//                .onTrue(s_Flywheel.setTargetVelocity(5.0));
//            driver.y()
//                .onTrue(s_Flywheel.setTargetVelocity(50.0));
//            driver.a()
//                .onTrue(s_Flywheel.setTargetVelocity(350.0));
//            driver.b()
//                .onTrue(s_Flywheel.setTargetVelocity(750.0));
            driver.rightTrigger().whileTrue(s_Flywheel.shooting()
                .alongWith(
                    Commands.waitUntil(s_Flywheel::isReadyToShoot),
                    s_Indexer.setDesiredStateCmd(Indexer.IndexerState.RUNNING)
                ));
        }
    }



    private void configureTriggers() {
//        new FieldBoundingBox("AllianceSide",
//            new Translation2d(0.0, 0.0),
//            new Translation2d(Units.inchesToMeters(158.6), FieldConstants.FIELD_WIDTH),
//            s_Swerve::getPose
//        )
//            .onTrue(Commands.runOnce(() -> ShooterStructure.setMode(ShooterStructure.ShooterMode.TO_HUB)))
//            .onFalse(Commands.runOnce(() -> ShooterStructure.setMode(ShooterStructure.ShooterMode.FERRY)));
    }

    private void configureDriver() {
//        driver.rightTrigger()
//            .whileTrue(Commands.runOnce(() -> {
//                RobotState.getInstance().setAimState(RobotState.ShootingState.SHOOTING);
//            }))
//            .onFalse(Commands.runOnce(() -> {
//                RobotState.getInstance().setAimState(RobotState.ShootingState.NOTHING);
//            }));
//        driver.rightTrigger()
//            .whileTrue(
//                s_Intake.runIntake());
//
//        driver.leftTrigger()
//            .whileTrue(
//                s_Intake.outtakeIntake());
    }
}
