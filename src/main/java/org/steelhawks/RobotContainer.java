package org.steelhawks;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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

        driver.povRight().onTrue(
            Commands.runOnce(() -> {
                if (RobotState.getInstance().getShooterMode().equals(RobotState.ShooterMode.TO_HUB)) {
                    RobotState.getInstance().setShooterMode(RobotState.ShooterMode.FERRY);
                } else {
                    RobotState.getInstance().setShooterMode(RobotState.ShooterMode.TO_HUB);
                }
            }));

        driver.leftBumper()
                .whileTrue(ShootingCommands.shoot());

        driver.rightTrigger()
            .whileTrue(
                s_Intake.runIntake());

        driver.x()
            .onTrue(s_Intake.slamOut());

        driver.y()
            .onTrue(s_Intake.slamIn());

        driver.leftTrigger()
            .whileTrue(
                TeleopSwerve.overrideState());
    }
}
