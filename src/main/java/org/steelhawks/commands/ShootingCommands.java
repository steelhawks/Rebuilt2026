package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.vision.Vision;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.AllianceFlip;

public class ShootingCommands {

    public static Command shoot() {
        return Commands.sequence(
            Commands.runOnce(() ->
                RobotState.getInstance().setShootingState(ShootingState.SHOOTING)),
            Commands.runOnce(() -> {
                if (AllianceFlip.shouldFlip()
                    && RobotState.getInstance().getAimState().equals(RobotState.AimState.TO_HUB)
                ) {
                    Vision.whitelistTagIds(VisionConstants.RED_HUB_ONLY);
                } else {
                    Vision.whitelistTagIds(VisionConstants.BLUE_HUB_ONLY);
                }
            }),
            Commands.sequence(
                Commands.waitUntil(RobotContainer.s_Flywheel::isReadyToShoot),
                Commands.waitUntil(RobotContainer.s_Turret::atGoal),
                Commands.waitUntil(RobotContainer.s_Hood::atGoal),
                RobotContainer.s_Indexer.feed()
                    .deadlineFor(
                        Commands.waitUntil(() -> RobotContainer.s_Indexer.emptyFuel())
                            .andThen(Commands.waitSeconds(0.3))
                            .andThen(RobotContainer.s_Intake.agitate()
                                    .unless(() -> RobotState.getInstance().getAimState().equals(RobotState.AimState.FERRY))))
                    .repeatedly())
            .repeatedly())
            .finallyDo(() -> {
                RobotState.getInstance().setShootingState(ShootingState.NOTHING);
                Vision.whitelistTagIds(VisionConstants.ALL_ALLOWED_TAGS);
                CommandScheduler.getInstance().schedule(
                    RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));
            });
    }

    private static Command jamRecovery() {
        return Commands.waitUntil(RobotContainer.s_Indexer::isJammed)
            .andThen(
                Commands.sequence(
                    RobotContainer.s_Indexer.outtake().withTimeout(0.3),
                    RobotContainer.s_Indexer.feed().withTimeout(0.2))
                .repeatedly()
                    .until(() -> !RobotContainer.s_Indexer.isJammed()))
            .repeatedly();
    }
}
