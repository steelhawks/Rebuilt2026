package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.ShootingState;

public class ShootingCommands {

    public static Command shoot() {
        return Commands.sequence(
            Commands.runOnce(() ->
                RobotState.getInstance().setAimState(ShootingState.SHOOTING)),
            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(RobotContainer.s_Flywheel::isReadyToShoot),
                    Commands.waitUntil(RobotContainer.s_Turret::atGoal),
                    RobotContainer.s_Indexer.feed()
                ),
                jamRecovery()
            )
        ).finallyDo(() ->
            RobotState.getInstance().setAimState(ShootingState.NOTHING));
    }

    private static Command jamRecovery() {
        return Commands.waitUntil(RobotContainer.s_Indexer::isJammed)
            .andThen(
                Commands.sequence(
                    RobotContainer.s_Indexer.outtake().withTimeout(0.3),
                    RobotContainer.s_Indexer.runSpindexer().withTimeout(0.2)
                ).repeatedly().until(() -> !RobotContainer.s_Indexer.isJammed())
            ).repeatedly();
    }
}
