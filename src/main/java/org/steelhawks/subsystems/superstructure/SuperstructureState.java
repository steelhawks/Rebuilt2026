package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.*;
import org.steelhawks.commands.ShootingCommands;
import org.steelhawks.subsystems.beam.BeamIOInputsAutoLogged;
import org.steelhawks.subsystems.intake.IntakeConstants;

import java.util.EnumSet;
import java.util.Set;
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
    ),
    SPINDEXING(
      container -> {
          Command runCommand = null;
          BeamIOInputsAutoLogged inputs = new  BeamIOInputsAutoLogged();
          if (inputs.connected && inputs.detected) {
                if (container.s_Flywheel.isReadyToShoot() && container.s_Turret.atGoal() && container.s_Hood.atGoal()) {
                    runCommand = ShootingCommands.shoot();
                } else {
                    runCommand = container.s_Indexer.feed();
                }
          }
          return runCommand;
    }),
    SHOOTING(
            container -> {
                Command runCommand = null;
                if (RobotState.getInstance().getAimState() == RobotState.AimState.FERRY) {
                    runCommand = ShootingCommands.shoot();
                } else if (RobotState.getInstance().getAimState() == RobotState.AimState.TO_HUB) {
                    runCommand = ShootingCommands.shootWhileIntaking();
                }
                return runCommand;
            }
    ),
    IN_HOPPER(
            container -> {
                return Commands.none();
            }
    );
    private final Function<RobotContainer, Command>  commandSupplier;

    SuperstructureState(Function<RobotContainer, Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }


    public Command getCommand(RobotContainer container) {
        if (commandSupplier != null) {
            return Commands.none();
        }
        return commandSupplier.apply(container);
    }


    public Set<SuperstructureState> allowedNextStates() {
        Set<SuperstructureState> all = EnumSet.allOf(SuperstructureState.class);
        switch (this) {
            case INTAKING, SPINDEXING:
                all.remove(EnumSet.of(STOWED));
                return all;
            case STOWED:
                all.remove(EnumSet.of(INTAKING, SHOOTING, SPINDEXING));
                return all;
            case SHOOTING:
                all.remove(EnumSet.of(STOWED, INTAKING));
                return all;
            case IN_HOPPER:
                all.remove(EnumSet.of(STOWED, SHOOTING));
            default:
                return all;

        }
    }

    public double getAutoAlignHub(Pose2d pose) {
        return FieldConstants.Hub.HUB_CENTER.getDistance(pose.getTranslation());
    }


}
