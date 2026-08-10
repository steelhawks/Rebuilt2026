package org.steelhawks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.commands.HoodDefaultCommand;
import org.steelhawks.commands.ShootingCommands;
import org.steelhawks.commands.TeleopSwerve;
import org.steelhawks.commands.rumble.RumbleAPI;
import org.steelhawks.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.steelhawks.Constants.OIConstants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;


public class RobotContainer {

    private final RobotConfig config = RobotConfig.getConfig();
    private final Subsystems subsystems;
    private final ShootingCommands shootingCommands;
    private final Autos autos;

    private final CommandXboxController driver =
        new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Field", FieldConstants.FIELD_2D);
        RumbleAPI.register(driver);

        // Install the empty registry FIRST so that anything constructed below which
        // queries Subsystems.xIfPresent() during its own init (notably RobotState's
        // footprint extension, evaluated eagerly by Boundary.asTrigger) sees an
        // empty Optional rather than a not-installed exception.
        subsystems = Subsystems.init();

        Swerve swerve = config.createSwerve();
        subsystems.setSwerve(swerve);
        config.createPoseLink().ifPresent(subsystems::setPoseLink);
        config.createObjectVision().ifPresent(subsystems::setObjVision);
        config.createFlywheel().ifPresent(subsystems::setFlywheel);
        config.createTurret(RobotState.getInstance()::getEstimatedPose).ifPresent(subsystems::setTurret);
        config.createHood().ifPresent(subsystems::setHood);
        config.createIntake().ifPresent(subsystems::setIntake);
        config.createOldIntake().ifPresent(subsystems::setOldIntake);
        config.createIndexer().ifPresent(subsystems::setIndexer);
        config.createLEDMatrix().ifPresent(subsystems::setMatrix);
        config.createLEDStrip().ifPresent(subsystems::setStrip);

        boolean hasShootingDeps =
            Subsystems.intakeIfPresent().isPresent()
                && Subsystems.indexerIfPresent().isPresent()
                && Subsystems.flywheelIfPresent().isPresent()
                && Subsystems.turretIfPresent().isPresent();
        shootingCommands = hasShootingDeps ? new ShootingCommands(subsystems) : null;
        autos = config.hasAutos ? new Autos(subsystems, shootingCommands) : null;

        if (autos != null) {
            autos.init();
        }

        // Restart the Orange Pi vision service from the dashboard. Elastic renders
        // a published Command as a button. IfPresent because PoseLink is not on
        // every robot config.
        Subsystems.poseLinkIfPresent().ifPresent(
            link -> SmartDashboard.putData("Restart Vision", link.restartVisionCommand()));

        configureDrive(swerve);
        configureHood();
        configureIntake();
        configureShooting();
        Toggles.configureOverrides();
    }

    public Autos autos() {
        return autos;
    }

    private void configureDrive(Swerve swerve) {
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

        new Trigger(() -> {
            double x = RobotState.getInstance().getEstimatedPose().getX();
            if (AllianceFlip.shouldFlip()) {
                double boundary = AllianceFlip.applyX(FieldConstants.Trench.TRENCH_START_X);
                return x <= boundary;
            } else {
                return x >= FieldConstants.Trench.TRENCH_END_X;
            }
        })
            .onTrue(Commands.runOnce(() -> RobotState.getInstance().setAimState(AimState.FERRY)))
            .onFalse(Commands.runOnce(() -> RobotState.getInstance().setAimState(AimState.TO_HUB)));
    }

    private void configureHood() {
        Subsystems.hoodIfPresent().ifPresent(hood ->
            hood.setDefaultCommand(new HoodDefaultCommand(hood)));
    }

    private void configureIntake() {
        Subsystems.intakeIfPresent().ifPresent(intake -> {
            driver.rightBumper().whileTrue(intake.outtakeIntake());
            driver.rightTrigger()
                .whileTrue(
                    intake.runIntake().alongWith(intake.setDesiredStateCommand(Intake.State.INTAKE)));
            driver.x().onTrue(intake.setDesiredStateCommand(Intake.State.INTAKE));
            driver.y().onTrue(intake.setDesiredStateCommand(Intake.State.HOME));
            driver.a().onTrue(intake.setDesiredStateCommand(Intake.State.CENTER_OF_MOTION));
        });

        Subsystems.indexerIfPresent().ifPresent(indexer ->
            driver.povRight().whileTrue(indexer.outtake()));
    }

    private void configureShooting() {
        if (shootingCommands == null) return;

        Subsystems.flywheelIfPresent().ifPresent(flywheel ->
            new Trigger(flywheel::isReadyToShoot).and(driver.leftBumper())
                .whileTrue(RumbleAPI.steady().repeatedly()));

        driver.leftTrigger().whileTrue(shootingCommands.shootWhileIntaking());
        driver.leftBumper().whileTrue(shootingCommands.shoot());
    }
}
