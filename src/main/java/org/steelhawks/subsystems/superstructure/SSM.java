package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.wpilibj.Filesystem;
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

    private List<StateTransition>[] precomputedPaths;

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
//        setDesiredState(state, true, true);
    }
}
