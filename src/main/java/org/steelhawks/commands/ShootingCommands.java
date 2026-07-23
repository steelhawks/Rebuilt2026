package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.Subsystems;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.Swerve;

public class ShootingCommands {

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Flywheel flywheel;
    private final Turret turret;

    public ShootingCommands(Subsystems s) {
        this.swerve = Subsystems.swerve();
        this.intake = Subsystems.intake();
        this.indexer = Subsystems.indexer();
        this.flywheel = Subsystems.flywheel();
        this.turret = Subsystems.turret();
    }

    public Command shootWhileIntaking() {
        return shoot()
            .alongWith(
                intake.runIntake()
                    .alongWith(intake.setDesiredStateCommand(Intake.State.INTAKE)))
            .beforeStarting(swerve.toggleLowGear())
            .finallyDo(() -> CommandScheduler.getInstance().schedule(swerve.toggleNormal()));
    }

    public Command autonShootWhileIntaking() {
        return Commands.sequence(
            indexer.agitateSpindexer().withTimeout(0.4),
            shootWhileIntaking());
    }

    public Command autonShoot() {
        return Commands.sequence(
            shoot()
                .alongWith(
                    Commands.waitSeconds(1.5)
                        .andThen(intake.setDesiredStateCommand(Intake.State.HOME))));
    }

    public Command shoot() {
        return Commands.sequence(
            Commands.runOnce(() ->
                RobotState.getInstance().setShootingState(ShootingState.SHOOTING)),
            Commands.runOnce(() -> indexer.resetBeamState()),
            // AprilTag whitelisting now lives on the Orange Pi, driven by the
            // alliance sent in RobotOdomInputs (see subsystems/poselink). The Pi
            // rejects opposing-alliance tags continuously, so the old mid-shot
            // Vision.whitelistTagIds() narrowing is no longer needed here.
            Commands.waitUntil(() ->
                (flywheel.isReadyToShoot()
                    && !turret.isTraversing()
                    && turret.atGoal())
                    || RobotState.getInstance().getAimState().equals(AimState.FERRY)),
            indexer.feed()
                .alongWith(
                    Commands.waitUntil(() -> indexer.emptyFuel())
                        .andThen(Commands.waitSeconds(0.05))
                        .andThen(intake.agitate()
                            .onlyIf(() -> !intake.isRollersRunning())))
                .repeatedly()
        ).finallyDo(() -> {
            RobotState.getInstance().setShootingState(ShootingState.NOTHING);
            CommandScheduler.getInstance().schedule(
                intake.setDesiredStateCommand(Intake.State.INTAKE));
        });
    }
}
