package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.steelhawks.*;
import org.steelhawks.util.AllianceFlip;

import java.util.ArrayList;
import java.util.List;

public class ShooterTuner {

    private static ShooterTuner instance;

    public static ShooterTuner getInstance() {
        if (instance == null) instance = new ShooterTuner();
        return instance;
    }

    private static final double FLYWHEEL_RADIUS =
        switch (Constants.getRobot()) {
            case SIMBOT -> SubsystemConstants.SimBot.FLYWHEEL.flywheelRadius();
            case ALPHABOT -> SubsystemConstants.AlphaBot.FLYWHEEL.flywheelRadius();
            case OMEGABOT -> SubsystemConstants.OmegaBot.FLYWHEEL.flywheelRadius();
            default -> 0;
        };

    private static final double DEFAULT_SPIKE_THRESHOLD_AMPS = 30.0;
    private static final int BASELINE_WINDOW = 50;

    private enum ToFState {
        IDLE,
        WAITING_FOR_IMPACT
    }

    private final LoggedNetworkNumber manualHoodAngleDeg;
    private final LoggedNetworkNumber manualFlywheelSpeed;
    private final LoggedNetworkNumber manualDistance;
    private final LoggedNetworkNumber currentSpikeThreshold;
    private final LoggedNetworkBoolean useOdomDistance;
    private final LoggedNetworkBoolean feedBallTrigger;
    private final LoggedNetworkBoolean saveTrigger;
    private final LoggedNetworkBoolean hubImpactTrigger;

    private ToFState tofState = ToFState.IDLE;
    private double t0 = 0.0;
    private double lastTof = Double.NaN;

    private final double[] currentWindow = new double[BASELINE_WINDOW];
    private int windowIdx = 0;
    private double windowSum = 0.0;
    private boolean windowFull = false;

    private final List<double[]> savedPoints = new ArrayList<>();

    private ShooterTuner() {
        manualHoodAngleDeg = new LoggedNetworkNumber("ShooterTuner/Input/HoodAngleDeg", 80.0);
        manualFlywheelSpeed = new LoggedNetworkNumber("ShooterTuner/Input/FlywheelSpeed", 1.0);
        manualDistance = new LoggedNetworkNumber("ShooterTuner/Input/ManualDistanceMeters", 1.0);
        currentSpikeThreshold = new LoggedNetworkNumber("ShooterTuner/Input/CurrentSpikeThresholdA", DEFAULT_SPIKE_THRESHOLD_AMPS);
        useOdomDistance = new LoggedNetworkBoolean("ShooterTuner/Input/UseOdomDistance", true);
        feedBallTrigger = new LoggedNetworkBoolean("ShooterTuner/FeedBall", false);
        saveTrigger = new LoggedNetworkBoolean("ShooterTuner/SavePoint", false);
        hubImpactTrigger = new LoggedNetworkBoolean("ShooterTuner/HubImpact", false);
    }

    public void periodic() {
        double activeDistance = useOdomDistance.get()
            ? ShooterStructure.distanceToTarget(AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D))
            : manualDistance.get();

        double currentAmps = RobotContainer.s_Flywheel.getStatorCurrentAmps();
        updateCurrentBaseline(currentAmps);
        double baseline = getBaselineCurrent();

        if (Toggles.shooterTuningMode.get()) {
            double rps = ShooterStructure.linearToAngularVelocity(manualFlywheelSpeed.get(), FLYWHEEL_RADIUS);
            RobotContainer.s_Flywheel.setTargetVelocityForced(rps);
            if (RobotConfig.getConfig().hasHood && RobotContainer.s_Hood != null) {
                RobotContainer.s_Hood.setDesiredPositionForced(Rotation2d.fromDegrees(manualHoodAngleDeg.get()));
            }
        }

        if (feedBallTrigger.get()) {
            feedBallTrigger.set(false);
            tofState = ToFState.IDLE;
            lastTof = Double.NaN;
            CommandScheduler.getInstance().schedule(RobotContainer.s_Indexer.feed());
        }

        switch (tofState) {
            case IDLE -> {
                if (windowFull && (currentAmps - baseline) >= currentSpikeThreshold.get()) {
                    t0 = Timer.getFPGATimestamp();
                    tofState = ToFState.WAITING_FOR_IMPACT;
                }
            }
            case WAITING_FOR_IMPACT -> {
                if (hubImpactTrigger.get()) {
                    hubImpactTrigger.set(false);
                    lastTof = Timer.getFPGATimestamp() - t0;
                    tofState = ToFState.IDLE;
                }
            }
        }

        if (saveTrigger.get()) {
            saveTrigger.set(false);
            savedPoints.add(new double[]{activeDistance, manualHoodAngleDeg.get(), manualFlywheelSpeed.get(), lastTof});
            System.out.printf(
                "[ShooterTuner] Saved point #%d → dist=%.2fm, hood=%.1f°, flywheel=%.1f, ToF=%s%n",
                savedPoints.size(), activeDistance, manualHoodAngleDeg.get(), manualFlywheelSpeed.get(),
                Double.isNaN(lastTof) ? "N/A" : String.format("%.4fs", lastTof));
            logAllSavedPoints();
        }

        Logger.recordOutput("ShooterTuner/Live/DistanceMeters", activeDistance);
        Logger.recordOutput("ShooterTuner/Live/HoodAngleDeg", manualHoodAngleDeg.get());
        Logger.recordOutput("ShooterTuner/Live/FlywheelSpeed", manualFlywheelSpeed.get());
        Logger.recordOutput("ShooterTuner/Live/UsingOdom", useOdomDistance.get());
        Logger.recordOutput("ShooterTuner/Live/StatorCurrentAmps", currentAmps);
        Logger.recordOutput("ShooterTuner/Live/BaselineCurrentAmps", baseline);
        Logger.recordOutput("ShooterTuner/Live/CurrentDeltaAmps", currentAmps - baseline);
        Logger.recordOutput("ShooterTuner/Live/TofState", tofState.name());
        Logger.recordOutput("ShooterTuner/Live/LastTofSeconds", lastTof);
        logAllSavedPoints();
    }

    private void updateCurrentBaseline(double sample) {
        windowSum -= currentWindow[windowIdx];
        currentWindow[windowIdx] = sample;
        windowSum += sample;
        windowIdx = (windowIdx + 1) % BASELINE_WINDOW;
        if (windowIdx == 0) windowFull = true;
    }

    private double getBaselineCurrent() {
        int n = windowFull ? BASELINE_WINDOW : windowIdx;
        return n > 0 ? windowSum / n : 0.0;
    }

    private void logAllSavedPoints() {
        int n = savedPoints.size();
        double[] distances = new double[n];
        double[] hoods = new double[n];
        double[] flywheels = new double[n];
        double[] tofs = new double[n];

        for (int i = 0; i < n; i++) {
            distances[i] = savedPoints.get(i)[0];
            hoods[i] = savedPoints.get(i)[1];
            flywheels[i] = savedPoints.get(i)[2];
            tofs[i] = savedPoints.get(i)[3];
        }

        Logger.recordOutput("ShooterTuner/Saved/Distances", distances);
        Logger.recordOutput("ShooterTuner/Saved/HoodAngles", hoods);
        Logger.recordOutput("ShooterTuner/Saved/FlywheelSpeeds", flywheels);
        Logger.recordOutput("ShooterTuner/Saved/TimesOfFlight", tofs);
        Logger.recordOutput("ShooterTuner/Saved/Count", n);
    }
}
