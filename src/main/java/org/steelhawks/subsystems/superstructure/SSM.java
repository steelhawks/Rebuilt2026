package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;

import java.io.*;
import java.nio.file.FileSystem;
import java.util.*;

public class SSM {
    Map<SuperstructureState, List<StateTransition>> graph = new HashMap<>();
    private final Set<SuperstructureState> states = new HashSet<>();
    private final List<StateTransition> transitions = new ArrayList<>();
    private SuperstructureState currentState;
    private SuperstructureState desiredState;
    private SuperstructureState futureDesiredState;
    private RobotContainer container;
    private boolean transitioning = false;
    private int lastClosestFace = 0;

    private List<StateTransition>[][] precomputedPaths;

    private Map<String, Double> transitionCostMap = new HashMap<>();

    private String getTransitionKey(SuperstructureState to, SuperstructureState from) {
        return from.name() + "--> "  + to.name();
    }

    private double getTransitionCost(SuperstructureState from, SuperstructureState to) {
        return transitionCostMap.getOrDefault(getTransitionKey(from, to), 1.0);
    }

    private void loadTransitionCosts() throws FileNotFoundException {
        transitionCostMap.clear();
        File costFile = new File(Filesystem.getDeployDirectory(), "transition-costs.txt");
        try (BufferedReader br = new BufferedReader(new FileReader(costFile))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] tokens = line.split(",");
                if (tokens.length == 3) {
                    String key = tokens[0] + "-->" +  tokens[1];
                    double duration = Double.parseDouble(tokens[2]);
                    transitionCostMap.put(key, duration);
                    Logger.recordOutput("Using Transition Costs", true);
                }
            }
        } catch (IOException e) {
            Logger.recordOutput("Superstructure/Error", "Failed to load transition-costs.txt" + e.getMessage());
            throw new RuntimeException(e);
        }
    }

    public SSM(RobotContainer container) throws FileNotFoundException {
        this.currentState = null;
        this.desiredState = null;
        this.futureDesiredState = null;
        this.container = container;
        loadTransitionCosts();
        autoGenerateTransitions();
        precomputeAllPaths();
    }

    public void setTransitioning(boolean transitioning) {
        this.transitioning = transitioning;
    }

    public void addState(SuperstructureState state) {
        states.add(state);
    }

    public void addTransition(StateTransition transition) {
        states.add(transition.getFromState());
        states.add(transition.getToState());
        transitions.add(transition);
    }

    public SuperstructureState getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SuperstructureState state) {
        if (!states.contains(currentState)) {
            throw new IllegalArgumentException(String.format("Superstructure/State %s not found", currentState));
        }
        this.currentState = state;
    }

    public SuperstructureState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SuperstructureState state) {
        setDesiredState(state, true, true);
    }

    public void setDesiredState(SuperstructureState state, boolean setFuture, boolean wipeFuture) {
        if (currentState == null && desiredState != null) {
            if (setFuture) setDesiredState(state);
            return;
        }
        if (!states.contains(state)) {
            throw new IllegalArgumentException(String.format("Superstructure/State %s not found", state));
        }
        if (ModalSuperstructureTriggers.isScoreableFuel(state)) {
            if (setFuture) setDesiredState(state);
            return;
        }
        desiredState = state;
        if (!DriverStation.isAutonomous() && wipeFuture) futureDesiredState = null;
        continueTransition();
    }

    private double futureDesiredStateTime = 0;

    public SuperstructureState getFutureDesiredState() {
        if (futureDesiredState != null && Timer.getFPGATimestamp() - futureDesiredStateTime > 3) {
            futureDesiredState = null;
        }
        return futureDesiredState;
    }

    private void setFutureDesiredState(SuperstructureState state) {
        if (!states.contains(state)) {
            throw new IllegalArgumentException("State not registered: " + state);
        }
        futureDesiredState = state;
        futureDesiredStateTime = Timer.getFPGATimestamp();
    }

    public void wipeFutureDesiredState() {
        futureDesiredState = null;
    }

    public void applyFutureDesiredState() {
        if (getFutureDesiredState() != null) {
            Logger.recordOutput("LastApplyFutureDesiredState", Timer.getFPGATimestamp());
            setDesiredState(futureDesiredState);
            futureDesiredState = null;
        }
    }

    public boolean isStable() {
        return !transitioning;
    }

    public void autoGenerateTransitions() {
        for (SuperstructureState from : SuperstructureState.values()) {
            for (SuperstructureState to : from.allowedNextStates()) {
                double cost = getTransitionCost(from, to);
                addTransition(
                        new StateTransition(from, to, () -> to.getCommand(container), cost, false)
                );
            }
        }
    }

    private void precomputeAllPaths() {
        int numStates = SuperstructureState.values().length;
        precomputedPaths = new ArrayList[numStates][numStates];

        for (SuperstructureState from : SuperstructureState.values()) {
            for (SuperstructureState to : SuperstructureState.values()) {
                if (from.equals(to)) {
                    precomputedPaths[from.ordinal()][to.ordinal()] = new ArrayList<>();
                } else {
                    precomputedPaths[from.ordinal()][to.ordinal()] = computeTransitionPath(from, to);
                }
            }
        }
    }

    private List<StateTransition> getPrecomputedPath(
            SuperstructureState from, SuperstructureState to) {
        return precomputedPaths[from.ordinal()][to.ordinal()];
    }

    public List<StateTransition> computeTransitionPath(SuperstructureState from, SuperstructureState to) {
        graph = new HashMap<>();
        for (SuperstructureState state : states) {
            graph.put(state, new ArrayList<>());
        }
        for (StateTransition transition : transitions) {
            if (!transition.hasCollision()) {
                graph.get(transition.getFromState()).add(transition);
            }
        }
        AStarSolver<SuperstructureState> solver = new AStarSolver<>();
        List<SuperstructureState> statePath =
                solver.solve(
                        from,
                        to,
                        (current, goal) -> current != null && current.equals(goal) ? 0 : 1,
                        state -> {
                            List<AStarSolver.Edge<SuperstructureState>> neighbors =
                                    new ArrayList<>();
                            for (StateTransition transition : graph.get(state)) {
                                neighbors.add(
                                        new AStarSolver.Edge<>(
                                                transition.getToState(), transition.getTransitionTime()));
                            }
                            return neighbors;
                        });
        if (statePath == null) return null;
        List<StateTransition> path = new ArrayList<>();
        for (int i = 0; i < statePath.size() - 1; i++) {
            SuperstructureState currentFrom = statePath.get(i);
            SuperstructureState currentTo = statePath.get(i + 1);
            Optional<StateTransition> transition =
                    graph.get(currentFrom).stream()
                            .filter(t -> t.getToState().equals(currentTo))
                            .findFirst();
            if (transition.isPresent()) {
                path.add(transition.get());
            } else {
                throw new IllegalStateException(" No transition found from " + currentFrom + " to " + currentTo);
            }
        }
        return path;
    }

    public List<StateTransition> computeDynamicTransitionPath(
            SuperstructureState from, SuperstructureState to) {
        Map<SuperstructureState, List<StateTransition>> dynamicGraph = new HashMap<>();
        for (SuperstructureState state : states) {
            dynamicGraph.put(state, new ArrayList<>());
        }
        for (StateTransition t : transitions) {
            if (!t.hasCollision()) {
                dynamicGraph.get(t.getFromState()).add(t);
            }
        }
        AStarSolver<SuperstructureState> solver = new AStarSolver<>();
        List<SuperstructureState> statePath =
                solver.solve(
                        from,
                        to,
                        (current, goal) -> current.equals(goal) ? 0 : 1,
                        state -> {
                            List<AStarSolver.Edge<SuperstructureState>> neighbors =
                                    new ArrayList<>();
                            for (StateTransition t : dynamicGraph.get(state)) {
                                neighbors.add(
                                        new AStarSolver.Edge<>(
                                                t.getToState(), t.getTransitionTime()));
                            }
                            return neighbors;
                        });
        if (statePath == null) return null;
        List<StateTransition> transitionPath = new ArrayList<>();
        for (int i = 0; i < statePath.size() - 1; i++) {
            SuperstructureState currentFrom = statePath.get(i);
            SuperstructureState currentTo = statePath.get(i + 1);
            Optional<StateTransition> transition =
                    dynamicGraph.get(currentFrom).stream()
                            .filter(t -> t.getToState().equals(currentTo))
                            .findFirst();
            if (transition.isPresent()) {
                transitionPath.add(transition.get());
            } else {
                throw new IllegalStateException(
                        "No dynamic transition found from " + currentFrom + " to " + currentTo);
            }
        }
        return transitionPath;
    }

    public void continueTransition() {
        if (transitioning) return;

        if (currentState == null) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            desiredState.getCommand(container),
                            new InstantCommand(
                                    () -> {
                                        setCurrentState(desiredState);
                                        transitioning = false;
                                        if (!currentState.equals(desiredState)) {
                                            continueTransition();
                                        }
                                    }))
                            .withName("Superstructure Move")
                            .ignoringDisable(true)
            );

            return;
        }

        if (currentState.equals(desiredState)) {
            return;
        }

        List<StateTransition> path = getPrecomputedPath(desiredState, currentState);
        if (path == null && path.isEmpty()) {
            setDesiredState(desiredState);
            return;
        }

        StateTransition nextTransitionTemp = path.get(0);

        final StateTransition nextTransition = nextTransitionTemp;
        Command transitionCommand = nextTransition.getCommand();
        transitioning = true;
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        desiredState.getCommand(container),
                        new InstantCommand(
                                () -> {
                                    setCurrentState(desiredState);
                                    transitioning = false;
                                    if (!currentState.equals(desiredState)) {
                                        continueTransition();
                                    }
                                }))
                        .withName("Superstructure Move")
                        .ignoringDisable(true)
        );
    }



    public Command BuildCharacterizationCommand() {
        Command overallSequence = Commands.none();
        desiredState = SuperstructureState.STOWED;
        File costFile = new File(Filesystem.getDeployDirectory(), "transition-costs.txt");
        for (StateTransition t : transitions) {
            final double[] startTime = new double[1];
            Command transitionSequence =
                    Commands.sequence(
                            new InstantCommand(
                                    () -> {
                                        currentState = t.getToState();
                                        desiredState = t.getToState();
                                        Logger.recordOutput(
                                                "Latest Characterization State",
                                                t.getFromState()
                                                        + " to "
                                                        + t.getToState());
                                    }),
                            new InstantCommand(
                                    () -> startTime[0] = Timer.getFPGATimestamp()),
                            Commands.sequence(
                                    t.getCommand(),
                                    new InstantCommand(
                                            () ->
                                                setCurrentState(
                                                        t
                                                                .getToState())))
                                    .withTimeout(5),
                            new InstantCommand(
                                    () -> {
                                        double duration =
                                                Timer.getFPGATimestamp() - startTime[0];
                                        String logMessage;
                                        if (!getCurrentState().equals(t.getToState())) {
                                            logMessage =
                                                    t.getFromState()
                                                    + ","
                                                    + t.getToState()
                                                    + ", 100";
                                        } else {
                                            logMessage =
                                                    t.getFromState()
                                                            + ","
                                                            + t.getToState()
                                                            + ","
                                                            + duration;
                                        }
                                        try (FileWriter fw =
                                                     new FileWriter(costFile, true);
                                             BufferedWriter bw = new BufferedWriter(fw);
                                             PrintWriter out = new PrintWriter(bw)) {
                                            out.println(logMessage);
                                        } catch (IOException ioe) {
                                            ioe.printStackTrace();
                                        }
                                    }))
                            .withTimeout(5);
            overallSequence = Commands.sequence(overallSequence, transitionSequence);
        }
        overallSequence = Commands.sequence(
                overallSequence,
                new InstantCommand(
                        () -> Logger.recordOutput("Characterization Complete", true)
                )
        );
        return overallSequence;
    }

}
