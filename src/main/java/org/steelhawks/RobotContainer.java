package org.steelhawks;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.oldintake.OldIntake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOTalonFX;
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

    public static LEDCommands LEDCommands;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    private Trigger runTechnicianScreen;
    private Trigger runRainbowLEDs;

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
//        s_Intake = null; // CAREFUL W THE INTAKE, THE GEAR RATIO HAS BEEN CHANGED TO THE ACCURATE ONE SO SETPOINTS ARE ALL CHANGED!!!
        s_OldIntake = config.createOldIntake().orElse(null);
        s_Indexer = config.createIndexer().orElse(null);

//        LEDCommands = new LEDCommands(s_Matrix);

        if (config.hasAutos) {
            Autos.init();
        }
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

        runTechnicianScreen = new Trigger(() -> Robot.isFirstRun() && DriverStation.isDisabled()).debounce(10)
            .whileTrue(s_Matrix.fireCommand(2, 2))
            .onFalse(s_Matrix.clearCommand());

        runRainbowLEDs = new Trigger(() -> !Robot.isFirstRun() && DriverStation.isDisabled())
            .whileTrue(s_Matrix.rainbowWaveCommand(5))
            .onFalse(s_Matrix.clearCommand());

        configureDriver();
    }

    private void configureDriver() {

//        driver.leftBumper().onTrue(s_Matrix.fireCommand(2, 2));
//        driver.rightBumper().onTrue(s_Matrix.clearCommand());
//
//        driver.x().onTrue(s_Swerve.zeroHeading());

//        driver.x().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.HOME));
//        driver.y().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));
//        driver.a().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.RETRACTED));
//
//        driver.leftBumper()
//            .whileTrue(ShootingCommands.shoot());
//
//        driver.rightTrigger()
//            .whileTrue(
//                s_Intake.runIntake());
//
//        driver.leftTrigger()
//            .whileTrue(
//                s_Intake.outtakeIntake());
    }
}
