package org.steelhawks;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.oldintake.OldIntake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.superstructure.ShooterTuner;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;
import org.steelhawks.util.DriverWarnings;
import org.steelhawks.util.geometry.RobotFootprint;

import java.awt.*;

public class RobotContainer {

    private final RobotConfig config = RobotConfig.getConfig();
    public static DriverWarnings warnings = DriverWarnings.getInstance();

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

    private static Trigger tooFarTrigger;
    private static Trigger tooCloseTrigger;
    private static Trigger noSolutionTrigger;

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

        if (config.hasLEDMatrix) {
            configureWarningTriggers();
        }

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));
        configureDriver();
        Toggles.configureOverrides();
//        ShooterTuner.getInstance();
    }

    private void configureDriver() {
        driver.povLeft().onTrue(s_Swerve.zeroHeading())
            .onTrue(new VibrateController(driver));

        if (config.hasIntake) {
            driver.x().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.HOME));
            driver.y().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));
            driver.a().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.RETRACTED));

            driver.rightTrigger()
                .whileTrue(
                    s_Intake.runIntake());
        }

        if (config.hasIntake && config.hasIndexer && config.hasIntake) {
            driver.leftBumper()
                .whileTrue(ShootingCommands.shoot());
        }

        if (config.hasIndexer) {
            driver.rightBumper()
                .whileTrue(s_Indexer.outtake());
        }

        driver.leftTrigger()
            .whileTrue(
                TeleopSwerve.overrideState()
                    .alongWith(new VibrateController(driver).repeatedly()));
    }

    private void configureWarningTriggers() {
        tooFarTrigger = warnings.tooFarAlert.asTrigger()
            .onTrue(Commands.sequence(
                Commands.parallel(
                    s_Matrix.flashCommand(Color.RED, 0.1, 1),
                    new VibrateController(1, 1, driver)
                ),
                s_Matrix.scrollingTextCommand("TOO FAR", Color.RED, 5),
                new WaitCommand(4),
                s_Matrix.clearCommand()
            ));

        tooCloseTrigger = warnings.tooCloseAlert.asTrigger()
            .onTrue(Commands.sequence(
                Commands.parallel(
                    s_Matrix.flashCommand(Color.YELLOW, 0.1, 1),
                    new VibrateController(0.5, 1, driver)
                ),
                s_Matrix.scrollingTextCommand("TOO CLOSE", Color.YELLOW, 5),
                new WaitCommand(5),
                s_Matrix.clearCommand()
            ));
    }
}
