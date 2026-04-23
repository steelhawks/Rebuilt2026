package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.intake.MagicIntake;
import org.steelhawks.subsystems.shooterSuperstructure.ShooterStructure;

import java.util.function.Function;

public enum SuperstructureState {
    INTAKING(
            container -> {
                Command moveCommand;
                moveCommand = container.s_Intake.setDesiredStateCommand(IntakeConstants.State.INTAKE);
                return moveCommand;
            }
    ),
    STOWED(
            container -> {
                Command moveCommand = null;
                if (container.s_Intake.getCurrentState() == IntakeConstants.State.INTAKE) {
                    if (RobotState.getInstance().getShootingState() != RobotState.ShootingState.SHOOTING ||
                        RobotState.getInstance().getShootingState() != RobotState.ShootingState.SHOOTING_MOVING ||
                        RobotState.getInstance().getShootingState() != RobotState.ShootingState.SHOOTING_STATIONARY) {
                        moveCommand = container.s_Intake.setDesiredStateCommand(IntakeConstants.State.HOME);
                    }
                }
                return moveCommand;
            }
    );



    private final Function<RobotContainer, Command>  commandSupplier;

    SuperstructureState(Function<RobotContainer, Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }
}
