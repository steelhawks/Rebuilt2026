package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.commands.rumble.RumbleAPI;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.vision.Vision;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AllianceFlip;

public class ShootingCommands {

    public static Command shootWhileIntaking() {
        return shoot()
            .alongWith(
                RobotContainer.s_Intake.runIntake()
                    .alongWith(RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE)))
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
            RobotContainer.s_Indexer.agitateSpindexer().withTimeout(0.4),
            shoot());
    }

    public static Command shoot() {
        return Commands.sequence(
            Commands.runOnce(() ->
                RobotState.getInstance().setShootingState(ShootingState.SHOOTING)),
            Commands.runOnce(() -> RobotContainer.s_Indexer.resetBeamState()),
            Commands.runOnce(() -> {
                if (AllianceFlip.shouldFlip()
                    && RobotState.getInstance().getAimState().equals(RobotState.AimState.TO_HUB)
                ) {
                    Vision.whitelistTagIds(VisionConstants.RED_TAGS);
                } else {
                    Vision.whitelistTagIds(VisionConstants.BLUE_TAGS);
                }
            }),

//            new ScheduleCommand(
//                RumbleAPI.staccato()
//                    .repeatedly()
//                    .until(RobotContainer.s_Vision::hasStableTag)
//                    .onlyIf(() -> !RobotContainer.s_Vision.hasStableTag())),

            Commands.sequence(
                Commands.waitUntil(() ->
                    RobotContainer.s_Flywheel.isReadyToShoot()
                        && RobotContainer.s_Turret.atGoal()
                        && RobotContainer.s_Hood.atGoal()),
                RobotContainer.s_Indexer.feedWithJamRecovery()
                    .alongWith(
                        Commands.waitUntil(() -> RobotContainer.s_Indexer.emptyFuel())
                            .andThen(Commands.waitSeconds(0.3))
                            .andThen(RobotContainer.s_Intake.feed()
                                .onlyIf(() -> !RobotContainer.s_Intake.isRollersRunning())))
                    .repeatedly())
            .repeatedly())
        .finallyDo(() -> {
            RobotState.getInstance().setShootingState(ShootingState.NOTHING);
            Vision.whitelistTagIds(VisionConstants.ALL_ALLOWED_TAGS);
            CommandScheduler.getInstance().schedule(
                RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));
        });
    }
}