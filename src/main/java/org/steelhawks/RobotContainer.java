package org.steelhawks;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.led.LEDMatrix;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.superstructure.SuperStructure;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.pivot.Pivot;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;
import org.steelhawks.util.FieldBoundingBox;

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
    public static Intake s_Intake = null;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);

        s_Swerve = config.createSwerve();
        s_LEDMatrix = config.createLEDMatrix().orElse(null);
        s_LEDStrip = config.createLEDStrip().orElse(null);
        if (config.hasVision) {
            s_Vision = config.createVision(s_Swerve::accept).orElse(null);
        }
        s_ObjVision = config.createObjectVision().orElse(null);
        s_Flywheel = config.createFlywheel().orElse(null);
        s_Turret = config.createTurret(s_Swerve::getPose).orElse(null);
        s_Pivot = config.createPivot().orElse(null);
        s_Intake = config.createIntake().orElse(null);

        if (config.hasAutos) {
            Autos.init();
        }
        if (Constants.getRobot() != RobotType.TEST_BOARD) {
            s_Swerve.setDefaultCommand(
                DriveCommands.joystickDrive(
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()));
            configureTriggers();
            configureDriver();
        }

        driver.x()
            .onTrue(
                s_Turret.setDesiredRotation(new Rotation2d(0.0)));
        driver.y()
            .onTrue(
                s_Turret.setDesiredRotation(new Rotation2d(Math.PI / 2.0)));
        driver.a()
            .onTrue(
                s_Turret.setDesiredRotation(new Rotation2d(Math.PI)));
        driver.b()
            .onTrue(
                s_Turret.setDesiredRotation(new Rotation2d(-Math.PI / 2.0)));
    }

    private boolean isHubActive() {
        String gameData = DriverStation.getGameSpecificMessage();
        return !gameData.isEmpty()
            && gameData.charAt(0) == 'B'
            && DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    private void configureTriggers() {
        new FieldBoundingBox("AllianceSide",
            new Translation2d(0.0, 0.0),
            new Translation2d(Units.inchesToMeters(158.6), FieldConstants.FIELD_WIDTH),
            s_Swerve::getPose
        )
            .onTrue(Commands.runOnce(() -> SuperStructure.setMode(SuperStructure.ShooterMode.TO_HUB)))
            .onFalse(Commands.runOnce(() -> SuperStructure.setMode(SuperStructure.ShooterMode.FERRY)));
    }

    private void configureDriver() {}
}
