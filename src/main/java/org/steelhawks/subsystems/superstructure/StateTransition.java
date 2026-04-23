package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class StateTransition {
    // ALL OPTIONAL<S> types are SuperstructureState types
    private final SuperstructureState fromState;
    private final SuperstructureState toState;
    private final Supplier<Command> commandSupplier;
    private final double transitionTime;
    private final boolean isCollision;

    public StateTransition(
            SuperstructureState fromState,
            SuperstructureState toState,
            Supplier<Command> commandSupplier,
            double transitionTime,
            boolean isCollision
    ) {
        this.fromState = fromState;
        this.toState = toState;
        this.commandSupplier = commandSupplier;
        this.transitionTime = transitionTime;
        this.isCollision = isCollision;
    }

    public SuperstructureState getFromState() {
        return fromState;
    }

    public SuperstructureState getToState() {
        return toState;
    }

    public Command getCommand() {
        return commandSupplier.get();
    }

    public boolean hasCollision() {
        return isCollision;
    }

    public double getTransitionTime() {
        return transitionTime;
    }

    @Override
    public String toString() {
        return "Transition from " +  fromState + " to " + toState + "(time: " + transitionTime + "), Collision: " + isCollision;
    }

}
