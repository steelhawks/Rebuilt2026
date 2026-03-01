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
import org.steelhawks.subsystems.superstructure.ShooterTuner;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOTalonFX;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;
import org.steelhawks.util.geometry.RobotFootprint;

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

    public final RobotFootprint FOOTPRINT;

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

        FOOTPRINT =
            new RobotFootprint(
                RobotConstants.ROBOT_LENGTH_WITH_BUMPERS,
                RobotConstants.ROBOT_WIDTH_WITH_BUMPERS)
                .withExtension(new RobotFootprint.Extension(
                    "Intake",
                    Rotation2d.fromDegrees(0.0),
                    s_Intake::getPosition));
        if (config.hasAutos) {
            Autos.init();
        }
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));
        driver.x().onTrue(s_Swerve.zeroHeading());
        configureDriver();
        ShooterTuner.getInstance();
    }

    private void configureDriver() {
//        driver.x().onTrue(s_Swerve.zeroHeading());

        driver.x().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.HOME));
        driver.y().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));
        driver.a().onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.RETRACTED));

        driver.leftBumper()
            .whileTrue(ShootingCommands.shoot());

        driver.rightBumper()
            .whileTrue(s_Indexer.outtake());

        driver.rightTrigger()
            .whileTrue(
                s_Intake.runIntake());

        driver.leftTrigger()
            .whileTrue(
                TeleopSwerve.setDriveState(TeleopSwerve.DriveState.NORMAL)
                    .alongWith(new VibrateController(driver).repeatedly()));
    }
}
