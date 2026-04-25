package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.RobotState;

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
        return Commands.none();
    }

    private Command setStateCommand(SuperstructureState state, boolean setFuture, String name) {
        return Commands.none();
    }

    private Command setStateCommand(AtomicReference<SuperstructureState> state, boolean setFuture, String name) {
        return Commands.none();
    }

    public Command setAndWaitCommand(SuperstructureState state, String name) {
        return Commands.sequence(
                setStateCommand(state, name),
                new WaitUntilCommand(() -> stateMachine != null)
        );
    }

    public Command setFutureStateAndWaitCommand(SuperstructureState state, String name) {
        return Commands.sequence(
                setStateCommand(state, true, name),
                new WaitUntilCommand(() -> stateMachine != null)
        );
    }

    public Command commandToDesiredStateCommand() {
        return Commands.defer(
                () -> setAndWaitCommand(null, "Stage Desired State"),
                Set.of());
    }

    private Command createScoreHubCommand() {
        Command indexCommand = commandToDesiredStateCommand();
        Command scoreCommand = new InstantCommand(Commands::none);
        Command fullScoreSequence = Commands.sequence(indexCommand, scoreCommand);
        return new ConditionalCommand(
                fullScoreSequence,
                Commands.none(),
                () -> isScoreableFuel(null)).withName("HI");
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
                () -> fuelAtShooter.get() && container.s_Flywheel.isReadyToShoot()
        );
    }


}
