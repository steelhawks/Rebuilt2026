package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.steelhawks.*;

import java.util.ArrayList;
import java.util.List;

public class ShooterTuner {

    private static ShooterTuner instance;
    public static ShooterTuner getInstance() {
        if (instance == null) {
            instance = new ShooterTuner();

        }
        return instance;
    }

    // TODO: find a better way to get the appropriate constants into classes like this
    private static final double FLYWHEEL_RADIUS =
        switch (Constants.getRobot()) {
            case SIMBOT -> SubsystemConstants.SimBot.FLYWHEEL.flywheelRadius();
            case ALPHABOT -> SubsystemConstants.AlphaBot.FLYWHEEL.flywheelRadius();
            case OMEGABOT -> SubsystemConstants.OmegaBot.FLYWHEEL.flywheelRadius();
            default -> 0;
        };

    private ShooterTuner() {
        manualHoodAngleDeg = new LoggedNetworkNumber("ShooterTuner/Input/HoodAngleDeg", 25.0);
        manualFlywheelSpeed = new LoggedNetworkNumber("ShooterTuner/Input/FlywheelSpeed", 150.0);
        manualDistance = new LoggedNetworkNumber("ShooterTuner/Input/ManualDistanceMeters", 2.0);
        useOdomDistance = new LoggedNetworkBoolean("ShooterTuner/Input/UseOdomDistance", true);
        saveTrigger = new LoggedNetworkBoolean("ShooterTuner/SavePoint", false);
    }

    private final LoggedNetworkNumber manualHoodAngleDeg;
    private final LoggedNetworkNumber manualFlywheelSpeed;
    private final LoggedNetworkNumber manualDistance;
    private final LoggedNetworkBoolean useOdomDistance;
    // set this to true on dashboard to save a point, auto resets
    private final LoggedNetworkBoolean saveTrigger;

    private final List<double[]> savedPoints =
        new ArrayList<>(); // [distance, hoodAngle, flywheelSpeed]

    public void periodic() {
        double activeDistance = useOdomDistance.get()
            ? ShooterStructure.distanceToTarget(FieldConstants.Hub.HUB_CENTER_3D)
            : manualDistance.get();
        Logger.recordOutput("ShooterTuner/Live/DistanceMeters", activeDistance);
        Logger.recordOutput("ShooterTuner/Live/HoodAngleDeg", manualHoodAngleDeg.get());
        Logger.recordOutput("ShooterTuner/Live/FlywheelSpeed", manualFlywheelSpeed.get());
        Logger.recordOutput("ShooterTuner/Live/UsingOdom", useOdomDistance.get());
        // run testing
        if (Toggles.shooterTuningMode.get()) {
            double rps = ShooterStructure.linearToAngularVelocity(
                manualFlywheelSpeed.get(), FLYWHEEL_RADIUS);
            RobotContainer.s_Flywheel.setTargetVelocityForced(rps);
            if (RobotConfig.getConfig().hasHood && RobotContainer.s_Hood != null) {
                RobotContainer.s_Hood.setDesiredPositionForced(Rotation2d.fromDegrees(manualHoodAngleDeg.get()));
            }
        }
        if (saveTrigger.get()) {
            saveTrigger.set(false); // auto reset
            double[] point = {activeDistance, manualHoodAngleDeg.get(), manualFlywheelSpeed.get()};
            savedPoints.add(point);
            logAllSavedPoints();
            System.out.printf(
                "[ShooterTuner] Saved point #%d -> dist=%.2fm, hood=%.1fdeg, flywheel=%.1f%n",
                savedPoints.size(), point[0], point[1], point[2]
            );
        }
        logAllSavedPoints();
    }

    private void logAllSavedPoints() {
        double[] distances = new double[savedPoints.size()];
        double[] hoods = new double[savedPoints.size()];
        double[] flywheels = new double[savedPoints.size()];

        for (int i = 0; i < savedPoints.size(); i++) {
            distances[i] = savedPoints.get(i)[0];
            hoods[i] = savedPoints.get(i)[1];
            flywheels[i] = savedPoints.get(i)[2];
        }
        Logger.recordOutput("ShooterTuner/Saved/Distances", distances);
        Logger.recordOutput("ShooterTuner/Saved/HoodAngles", hoods);
        Logger.recordOutput("ShooterTuner/Saved/FlywheelSpeeds", flywheels);
        Logger.recordOutput("ShooterTuner/Saved/Count", savedPoints.size());
    }
}