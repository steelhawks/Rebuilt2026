package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.vision.Vision;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AllianceFlip;

public class ShootingCommands {

    public static Command shootWhileIntaking() {
        return shoot()
            .alongWith(
                RobotContainer.s_Intake.runIntake()
                    .alongWith(RobotContainer.s_Intake.setDesiredStateCommand(Intake.State.INTAKE)))
            .beforeStarting(RobotContainer.s_Swerve.toggleLowGear())
            .finallyDo(() -> CommandScheduler.getInstance().schedule(RobotContainer.s_Swerve.toggleNormal()));
    }

    public static Command autonShootWhileIntaking() {
        return Commands.sequence(
            RobotContainer.s_Indexer.agitateSpindexer().withTimeout(0.4),
            shootWhileIntaking());
    }

    public static Command autonShoot() {
        return Commands.sequence(
//            RobotContainer.s_Indexer.agitateSpindexer().withTimeout(0.4),
            shoot()
                .alongWith(
                    Commands.waitSeconds(1.5)
                        .andThen(RobotContainer.s_Intake.setDesiredStateCommand(Intake.State.HOME))));
    }

    public static Command shoot() {
        return Commands.sequence(
            Commands.runOnce(() ->
                RobotState.getInstance().setShootingState(ShootingState.SHOOTING)),
            Commands.runOnce(() -> RobotContainer.s_Indexer.resetBeamState()),
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
                (RobotContainer.s_Flywheel.isReadyToShoot()
                    && !RobotContainer.s_Turret.isTraversing()
                    && RobotContainer.s_Turret.atGoal())
                    || RobotState.getInstance().getAimState().equals(AimState.FERRY)),
            RobotContainer.s_Indexer.feed()
                .alongWith(
                    Commands.waitUntil(() -> RobotContainer.s_Indexer.emptyFuel())
                        .andThen(Commands.waitSeconds(0.05))
                        .andThen(RobotContainer.s_Intake.agitate()
                            .onlyIf(() -> !RobotContainer.s_Intake.isRollersRunning())))
                .repeatedly()
        ).finallyDo(() -> {
            RobotState.getInstance().setShootingState(ShootingState.NOTHING);
            Vision.whitelistTagIds(VisionConstants.ALL_ALLOWED_TAGS);
            CommandScheduler.getInstance().schedule(
                RobotContainer.s_Intake.setDesiredStateCommand(Intake.State.INTAKE));
        });
    }
}