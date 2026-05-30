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
import org.steelhawks.subsystems.vision.Vision;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AllianceFlip;

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
            Commands.runOnce(() -> {
                if (AllianceFlip.shouldFlip()
                    && RobotState.getInstance().getAimState().equals(AimState.TO_HUB)
                ) {
                    Vision.whitelistTagIds(VisionConstants.RED_TAGS);
                } else {
                    Vision.whitelistTagIds(VisionConstants.BLUE_TAGS);
                }
            }),
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
            Vision.whitelistTagIds(VisionConstants.ALL_ALLOWED_TAGS);
            CommandScheduler.getInstance().schedule(
                intake.setDesiredStateCommand(Intake.State.INTAKE));
        });
    }
}
