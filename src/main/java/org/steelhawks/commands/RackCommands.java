package org.steelhawks.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.subsystems.intake.Intake;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class RackCommands {

    /**
     *@param intake
     * @return Command */
    public Command retract(Intake intake) {
        return runOnce(intake::retract, intake).andThen(Commands.run(() -> {
        }, intake)).until(intake::atTargetPosition).withName("Retract Intake");
    }

    /**
     *@param intake
     * @return Command */

    public Command intakeGamePiece(Intake intake) {
        return runOnce(() -> {
            intake.extendToIntake();
            intake.intake();
        }, intake)
                .andThen(Commands.run(() -> {}, intake))
                .finallyDo(intake::stopRollers)
                .withName("Intake Game Piece");
    }
}
