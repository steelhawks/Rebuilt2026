package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.steelhawks.commands.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.superstructure.ShooterSuperstructure;
import org.steelhawks.subsystems.led.LEDMatrix;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.superstructure.turret.TurretIOTalonFX;
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
    public static ShooterSuperstructure s_Superstructure = null;
    public static Intake s_Intake = null;
    public static Turret s_Turret = null;

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
//        s_Superstructure = config.createShooterSuperStructure().orElse(null);
        s_Intake = config.createIntake().orElse(null);


        s_Turret = new Turret(new TurretIOTalonFX(new RobotConfig.CANBus("")), s_Swerve::getPose);

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
                s_Turret.setDesiredState(new Rotation2d(0.0)));
        driver.y()
            .onTrue(
                s_Turret.setDesiredState(new Rotation2d(Math.PI / 2.0)));
        driver.a()
            .onTrue(
                s_Turret.setDesiredState(new Rotation2d(Math.PI)));
        driver.b()
            .onTrue(
                s_Turret.setDesiredState(new Rotation2d(-Math.PI / 2.0)));
    }

    private void configureTriggers() {}

    private void configureDriver() {}
}
