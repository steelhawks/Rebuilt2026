package org.steelhawks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.commands.*;
import org.steelhawks.commands.rumble.RumbleAPI;
import org.steelhawks.subsystems.beam.BeamIOInputsAutoLogged;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.intake.MagicIntake;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.oldintake.OldIntake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.*;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.shooterSuperstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.shooterSuperstructure.hood.Hood;
import org.steelhawks.subsystems.shooterSuperstructure.turret.MagicTurret;
import org.steelhawks.subsystems.superstructure.FuelStateTracker;
import org.steelhawks.subsystems.superstructure.SSM;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;
import org.steelhawks.util.AllianceFlip;

import java.io.FileNotFoundException;

public class RobotContainer<N> {

    private final SSM stateMachine;
    private final RobotConfig config = RobotConfig.getConfig();
    public static FuelStateTracker gamePieceTracker = null;
    private BeamIOInputsAutoLogged inputs;

    public static LEDMatrix s_Matrix = null;
    public static Swerve s_Swerve = null;
    public static Vision s_Vision = null;
    public static ObjectVision s_ObjVision = null;
    public static Flywheel s_Flywheel = null;
//    public static Turret s_Turret = null;
    public static MagicTurret s_Turret = null;
    public static Hood s_Hood = null;
//    public static Intake s_Intake = null;
    public static MagicIntake s_Intake = null;
    public static OldIntake s_OldIntake = null;
    public static Indexer s_Indexer = null;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() throws FileNotFoundException {

        stateMachine = new SSM(this);

        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);
        RumbleAPI.register(driver);

        s_Matrix = config.createLEDMatrix().orElse(null);
        s_Swerve = config.createSwerve();
        s_Vision = config.createVision().orElse(null);
        s_Flywheel = config.createFlywheel().orElse(null);
        s_Turret = config.createMagicTurret(RobotState.getInstance()::getEstimatedPose).orElse(null);
        s_Hood = config.createHood().orElse(null);
        s_Intake = config.createMagicIntake().orElse(null);
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
        s_Hood.setDefaultCommand(new HoodDefaultCommand(s_Hood));
        configureDriver();
        Toggles.configureOverrides();
        LEDCommands.configureTriggers(driver.leftTrigger());
    }

    private void configureDriver() {
        new Trigger(() -> s_Flywheel.isReadyToShoot()).and(driver.leftBumper())
            .whileTrue(RumbleAPI.steady().repeatedly());

        new Trigger(() -> {
            double x = RobotState.getInstance().getEstimatedPose().getX();
            double boundary = AllianceFlip.applyX(FieldConstants.Trench.TRENCH_END_X);
            return AllianceFlip.shouldFlip() ? x <= boundary : x >= boundary;
        })
            .onTrue(Commands.runOnce(() -> RobotState.getInstance().setAimState(AimState.FERRY)))
            .onFalse(Commands.runOnce(() -> RobotState.getInstance().setAimState(AimState.TO_HUB)));

        driver.povLeft().onTrue(s_Swerve.zeroHeading())
            .onTrue(RumbleAPI.steady());

        driver.povUp().onTrue(
            s_Flywheel.incrementVelocityFactor(0.03));

        driver.povDown().onTrue(
            s_Flywheel.incrementVelocityFactor(-0.03));

        driver.povLeft()
            .whileTrue(s_Indexer.outtake());

        driver.rightBumper()
            .whileTrue(s_Intake.outtakeIntake());

        driver.leftTrigger()
            .whileTrue(ShootingCommands.shootWhileIntaking());

        driver.leftBumper()
            .whileTrue(ShootingCommands.shoot());

        driver.rightTrigger()
            .whileTrue(
                s_Intake.runIntake().alongWith(s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE)));

        driver.x()
            .onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));

        driver.y()
            .onTrue(s_Intake.setDesiredStateCommand(IntakeConstants.State.HOME));

//        driver.x()
//            .onTrue(s_Turret.setDesiredRotation(Rotation2d.kPi));
//        driver.y()
//            .onTrue(s_Turret.setDesiredRotation(Rotation2d.kZero));
//        driver.a()
//            .onTrue(s_Turret.setDesiredRotation(Rotation2d.kPi.div(2.0)));
    }


    public SSM getStateMachine() {
        return stateMachine;
    }

    public Flywheel getFlywheel() {
        return s_Flywheel;
    }
}
