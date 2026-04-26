package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;
import org.steelhawks.commands.ShootingCommands;
import org.steelhawks.subsystems.shooterSuperstructure.flywheel.Flywheel;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ModalSuperstructureTriggers {
    private final RobotContainer container;
    private final SSM stateMachine;
    private final FuelStateTracker fuelTracker;

    private final AtomicReference<SuperstructureState> latestFuelState = new AtomicReference<>(
            SuperstructureState.IN_HOPPER
    );

    private final AtomicBoolean fuelAtShooter =  new AtomicBoolean(false);
    private final AtomicReference<RobotState.AimState> selectedAimstate = new AtomicReference<>(RobotState.AimState.TO_HUB);
    private final AtomicBoolean enableAutoStow = new AtomicBoolean(false);
    private final AtomicBoolean enableHubSnap =  new AtomicBoolean(true);

    public ModalSuperstructureTriggers(RobotContainer robotContainer, SSM stateMachine, FuelStateTracker fuelTracker) {
        this.container = robotContainer;
        this.stateMachine = stateMachine;
        this.fuelTracker = fuelTracker;

        configureTriggers();
    }

    public AtomicReference<SuperstructureState> getLatestFuelState() {
        return latestFuelState;
    }

    public static boolean isScoreableFuel(SuperstructureState fuelState) {
        return fuelState == SuperstructureState.SPINDEXING
                || fuelState == SuperstructureState.SHOOTING
                || fuelState == SuperstructureState.IN_HOPPER;
    }

    private boolean isCloseToHub(Pose2d pose) {
        Translation2d robotTranslation = pose.getTranslation();
        double distanceToHub =  FieldConstants.Hub.HUB_CENTER.getDistance(robotTranslation);
        return distanceToHub < 0.3;
    }

    private Command setStateCommand(SuperstructureState state, String name) {
        return new InstantCommand(
                () -> stateMachine.setDesiredState(state)
        ).withName(name);
    }

    private Command setStateCommand(SuperstructureState state, boolean setFuture, String name) {
        return new InstantCommand(
                () -> stateMachine.setDesiredState(state, setFuture, true)
        ).withName(name);
    }

    private Command setStateCommand(AtomicReference<SuperstructureState> state, boolean setFuture, String name) {
        return new InstantCommand(
                () -> stateMachine.setDesiredState(state.get(), setFuture, true)
        ).withName(name);
    }

    public Command setAndWaitCommand(SuperstructureState state, String name) {
        return Commands.sequence(
                setStateCommand(state, name),
                new WaitUntilCommand(() -> container.getStateMachine().getDesiredState() != null
                && stateMachine.getCurrentState().equals(state)));
    }

    public Command setFutureStateAndWaitCommand(SuperstructureState state, String name) {
        return Commands.sequence(
                setStateCommand(state, true, name),
                new WaitUntilCommand(() -> (container.getStateMachine().getDesiredState() != null
                && stateMachine.getDesiredState().equals(state)) ||
                        (container.getStateMachine().getCurrentState() != null
                        && container.getStateMachine().getCurrentState().equals(state)))
        );
    }

    public Command commandToDesiredStateCommand() {
        return Commands.defer(
                () -> setAndWaitCommand(stateMachine.getDesiredState(), "Stage Desired State"),
                Set.of());
    }

    private Command createScoreHubCommand() {
        Command indexCommand = commandToDesiredStateCommand();
        Command scoreCommand = new InstantCommand((Runnable) ShootingCommands.shoot());
        Command fullScoreSequence = Commands.sequence(indexCommand, scoreCommand);
        return new ConditionalCommand(
                fullScoreSequence,
                Commands.none(),
                () -> isScoreableFuel(stateMachine.getCurrentState())).withName("Superstructure Score");
    }


    private Command createExhaustFuelCommand() {
        return Commands.none();
    }

    private Command createSpindexerToShooterCommand() {
        return new SequentialCommandGroup(
                Commands.none()
        );
    }

    private Command createIntakingWhileSpindexerToShooterCommand() {
        return new SequentialCommandGroup(
                Commands.none()
        );
    }




    private void configureTriggers() {
        Trigger indexFuelIfShooterReady = new Trigger(
                () -> fuelAtShooter.get() && (container.getFlywheel().isReadyToShoot() && container.getStateMachine().getDesiredState() == SuperstructureState.SPINDEXING) ||
                        (container.getFlywheel().isReadyToShoot() && container.getStateMachine().getDesiredState() == SuperstructureState.SHOOTING)
        );


    }


}
