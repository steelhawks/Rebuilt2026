package org.steelhawks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.led.LEDMatrix;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.led.LEDStrip;
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
    public static Intake s_Intake = null;
    public static Flywheel s_Flywheel = null;
    public static Turret s_Turret = null;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);


    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Rebuilt/Field", FieldConstants.FIELD_2D);



        s_Swerve = config.createSwerve();

        s_LEDMatrix = config.hasLEDMatrix ? config.createLEDMatrix().orElseThrow() : null;
        s_LEDStrip = config.hasLEDStrip ? config.createLEDStrip().orElseThrow() : null;
        s_Vision = config.hasVision ? config.createVision(s_Swerve::accept).orElseThrow() : null;
        s_ObjVision = config.hasObjectVision ? config.createObjectVision().orElseThrow() : null;
        s_Intake = config.hasIntake ? config.createIntake().orElseThrow() : null;
        s_Flywheel = config.hasFlywheel ? config.createFlywheel().orElseThrow() : null;
        s_Turret = config.hasTurret ? config.createTurret(RobotState.getInstance()::getEstimatedPose).orElseThrow() : null;

        if (config.hasAutos) {
            Autos.init();
        }



        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

        configureTriggers();
        configureDriver();

    }

    private void configureTriggers() {}

    public Command spinUp(double velocityRadPerSec) {
        return Commands.sequence(
                Commands.runOnce(() -> s_Flywheel.runAtVelocity(velocityRadPerSec), s_Flywheel),
                Commands.waitUntil(s_Flywheel::isRampComplete)
        ).withName("Flywheel.spinUp");
    }

    private void configureDriver() {}
}
