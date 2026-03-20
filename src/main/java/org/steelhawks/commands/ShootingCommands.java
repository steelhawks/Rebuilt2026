package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.subsystems.intake.IntakeConstants;

public class ShootingCommands {

    public static Command shoot() {

        return Commands.sequence(
            Commands.runOnce(() ->
                RobotState.getInstance().setAimState(ShootingState.SHOOTING)),
            Commands.sequence(
                Commands.waitUntil(RobotContainer.s_Flywheel::isReadyToShoot),
//                Commands.waitUntil(RobotContainer.s_Turret::atGoal),
//                Commands.waitUntil(RobotContainer.s_Hood::atGoal),
                RobotContainer.s_Indexer.feed()
                    .deadlineFor(RobotContainer.s_Intake.agitate()).repeatedly()
//                    .alongWith(Commands.sequence(
//                        RobotContainer.s_Intake.slamIn(),
//                        RobotContainer.s_Intake.slamOut()
//                    ).repeatedly())
//                    .until(RobotContainer.s_Indexer::isJammed),
//                jamRecovery())
                )
            .repeatedly())
            .finallyDo(() -> {
                RobotState.getInstance().setAimState(ShootingState.NOTHING);
                CommandScheduler.getInstance().schedule(RobotContainer.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE));
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
