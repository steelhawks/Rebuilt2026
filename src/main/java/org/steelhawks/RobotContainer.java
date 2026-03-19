package org.steelhawks;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.oldintake.OldIntake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;

public class RobotContainer {

    private final RobotConfig config = RobotConfig.getConfig();

    public static LEDMatrix s_Matrix = null;
    public static Swerve s_Swerve = null;
    public static Vision s_Vision = null;
    public static ObjectVision s_ObjVision = null;
    public static Flywheel s_Flywheel = null;
    public static Turret s_Turret = null;
    public static Hood s_Hood = null;
    public static Intake s_Intake = null;
    public static OldIntake s_OldIntake = null;
    public static Indexer s_Indexer = null;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);

        s_Matrix = config.createLEDMatrix().orElse(null);
        s_Swerve = config.createSwerve();
        s_Vision = config.createVision().orElse(null);
        s_Flywheel = config.createFlywheel().orElse(null);
        s_Turret = config.createTurret(RobotState.getInstance()::getEstimatedPose).orElse(null);
        s_Hood = config.createHood().orElse(null);
        s_Intake = config.createIntake().orElse(null);
        s_OldIntake = config.createOldIntake().orElse(null);
        s_Indexer = config.createIndexer().orElse(null);

        if (config.hasAutos) {
            Autos.init();
        }
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));
        configureDriver();
        Toggles.configureOverrides();
        LEDCommands.configureTriggers(driver.leftTrigger());
    }

    private void configureDriver() {
        driver.povLeft().onTrue(s_Swerve.zeroHeading())
            .onTrue(new VibrateController(driver));


        driver.leftBumper()
                .whileTrue(ShootingCommands.shoot());

//        if (config.hasIntake) {
//            driver.rightTrigger()
//                .whileTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE)
//                    .andThen(s_Intake.runIntake())
//                    .finallyDo(() -> s_Intake.setDesiredState(IntakeConstants.State.HOME)));
//        }
        driver.rightTrigger()
            .whileTrue(
                s_Intake.slamOut()
                    .deadlineFor()

            )

//        if (config.hasTurret) {
//            driver.x()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromDegrees(0.0)));
//            driver.y()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromDegrees(90.0)));
//        }

        driver.leftTrigger()
            .whileTrue(
                TeleopSwerve.overrideState());
//
//            driver.a()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromDegrees(180.0)));
//
//            driver.b()
//                .onTrue(s_Turret.setDesiredRotation(Rotation2d.fromDegrees(-90.0)));
//        }

//        if (config.hasHood) {
//            driver.x()
//                .onTrue(s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(80.0)));
//
//            driver.y()
//                .onTrue(s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(60.0)));
//
//            driver.a()
//                .onTrue(s_Hood.setDesiredPositionCommand(Rotation2d.fromDegrees(45.0)));
//        }
    }
}
