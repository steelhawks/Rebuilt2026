package org.steelhawks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.oldintake.OldIntake;
import org.steelhawks.subsystems.led.LEDMatrix;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.led.LEDStrip;
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

    public static Swerve s_Swerve = null;
//    public static LEDMatrix s_LEDMatrix = null;
//    public static LEDStrip s_LEDStrip = null;
    public static Vision s_Vision = null;
    public static ObjectVision s_ObjVision = null;
    public static Flywheel s_Flywheel = null;
    public static Turret s_Turret = null;
//    public static Pivot s_Pivot = null;
    public static Intake s_Intake = null;
    public static OldIntake s_OldIntake = null;
    public static Indexer s_Indexer = null;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);

        s_Swerve = config.createSwerve();
//        s_LEDMatrix = config.createLEDMatrix().orElse(null);
//        s_LEDStrip = config.createLEDStrip().orElse(null);
        s_Vision = config.createVision().orElse(null);
//        s_ObjVision = config.createObjectVision().orElse(null);
//        s_Flywheel = config.createFlywheel().orElse(null);
        s_Flywheel = new Flywheel(new FlywheelIOTalonFX(new RobotConfig.CANBus("")));
        s_Turret = config.createTurret(RobotState.getInstance()::getEstimatedPose).orElse(null);
//        s_Pivot = config.createPivot().orElse(null);
        s_Intake = config.createIntake().orElse(null);
        s_OldIntake = config.createOldIntake().orElse(null);
        s_Indexer = config.createIndexer().orElse(null);

        if (config.hasAutos) {
            Autos.init();
        }
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));
        driver.x().onTrue(s_Swerve.zeroHeading());
        configureDriver();
    }

    private void configureDriver() {
        driver.leftBumper()
            .whileTrue(Commands.runOnce(() -> {
                RobotState.getInstance().setAimState(RobotState.ShootingState.SHOOTING);
            }))
            .onFalse(Commands.runOnce(() -> {
                RobotState.getInstance().setAimState(RobotState.ShootingState.NOTHING);
            }));
        driver.rightTrigger()
            .whileTrue(
                s_Intake.runIntake());

        driver.leftTrigger()
            .whileTrue(
                s_Intake.outtakeIntake());
    }
}
