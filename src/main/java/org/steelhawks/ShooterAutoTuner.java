package org.steelhawks;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.util.AllianceFlip;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Auto-tuner for the shooter LUT.
 *
 * Biases toward flat arcs and higher velocity over steep arcs and lower velocity.
 * Seeds each distance point at the floor of its valid hood range so the first shot
 * is always as flat as geometry allows. Short → velocity up first, hood up only as
 * last resort. Long → hood up first to bleed distance without adding arc time.
 *
 * Arc caps by zone:
 *   < 2m   → 80° max (geometry forces steep at point blank)
 *   2–4m   → 70° max (primary scoring range, keep flat)
 *   > 4m   → 65° max (long range, velocity does the work)
 *
 * With one practice match (~8-12 shots), seed from NYC offset so first shot
 * per distance is already close and minimal corrections are needed.
 */
public class ShooterAutoTuner {

    private static final double VELOCITY_STEP_MPS = 0.2;
    private static final double HOOD_STEP_DEG = 0.75;
    private static final double HOOD_MIN_DEG = 40.0;
    private static final double HOOD_MAX_DEG = 80.0;
    private static final double MIN_ANGLE_SEP_DEG = 0.5;

    // Flat arc zone caps
    private static final double CLOSE_RANGE_MAX_DEG = 80.0; // < 2m
    private static final double MID_RANGE_MAX_DEG = 70.0;   // 2-4m
    private static final double LONG_RANGE_MAX_DEG = 65.0;  // > 4m
    private static final double CLOSE_RANGE_THRESHOLD = 2.0;
    private static final double LONG_RANGE_THRESHOLD = 4.0;

    // Seed offset based on NYC experience — starting point for first shot per distance
    private static final double SEED_OFFSET_M = 0.594; // 1.95ft

    public record TunerPoint(double distanceM, double velocityMps, double hoodDeg) {}

    private final List<TunerPoint> confirmedPoints = new ArrayList<>();
    private double currentHoodDeg;
    private double currentVelocityMps;
    private boolean pointConfirmedThisDistance = false;

    // -------------------------------------------------------------------------
    // Public command factory
    // -------------------------------------------------------------------------

    /**
     * Tunes one shot at the robot's current distance.
     * Call repeatedly for each distance you want to cover.
     *
     * SHORT → velocity up first, hood up only if velocity alone can't fix it
     * LONG  → hood up first to bleed distance, velocity down only if needed
     * GOOD  → confirm point and log
     *
     * @param getShort  driver button: shot was short
     * @param getLong   driver button: shot was long
     * @param getGood   driver button: shot was good
     * @param shootCmd  command that fires one ball and ends
     */
    public Command tuneCurrentDistance(
        BooleanSupplier getShort,
        BooleanSupplier getLong,
        BooleanSupplier getGood,
        Command shootCmd
    ) {
        return Commands.sequence(
            Commands.runOnce(this::seedFromCurrentDistance),
            Commands.repeatingSequence(
                shootCmd,
                Commands.waitUntil(() -> getShort.getAsBoolean()
                    || getLong.getAsBoolean()
                    || getGood.getAsBoolean()),
                Commands.runOnce(() -> {
                    if (getGood.getAsBoolean()) {
                        confirmAndLog();
                    } else if (getShort.getAsBoolean()) {
                        adjustForShort();
                    } else {
                        adjustForLong();
                    }
                })
            ).until(this::lastPointConfirmed)
        );
    }

    public Command finishSession() {
        return Commands.runOnce(this::logSession);
    }

    // -------------------------------------------------------------------------
    // Seeding
    // -------------------------------------------------------------------------

    private void seedFromCurrentDistance() {
        pointConfirmedThisDistance = false;
        double dist = currentDistanceM();

        // Seed from NYC offset so first shot is already close
        double seedDist = Math.min(dist + SEED_OFFSET_M, ShooterStructure.getMaxShootDistance());
        var seed = ShooterStructure.Static.calculateShot(
            AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D),
            AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D),
            false,
            seedDist);

        // Start at the floor of the valid hood range for this distance — bias flat
        double zoneCap = zoneCap(dist);
        double neighborFloor = hoodFloor();
        double seedHood = Math.max(neighborFloor + MIN_ANGLE_SEP_DEG, HOOD_MIN_DEG);
        // clamp to zone cap but don't go below floor
        seedHood = Math.min(seedHood, zoneCap);

        currentVelocityMps = seed.exitVelocity();
        currentHoodDeg = seedHood;

        Logger.recordOutput("AutoTuner/CurrentDistance", dist);
        Logger.recordOutput("AutoTuner/SeededVelocity", currentVelocityMps);
        Logger.recordOutput("AutoTuner/SeededHoodDeg", currentHoodDeg);
        Logger.recordOutput("AutoTuner/ZoneCap", zoneCap);
    }

    // -------------------------------------------------------------------------
    // Adjustment logic — velocity first for short, hood first for long
    // -------------------------------------------------------------------------

    private void adjustForShort() {
        // Prefer velocity up — keeps arc flat
        // Only go to hood if velocity has already been bumped significantly
        // and shot is still short, implying geometry needs help
        double velHeadroom = 16.5 - currentVelocityMps; // reasonable max velocity
        if (velHeadroom > VELOCITY_STEP_MPS) {
            currentVelocityMps += VELOCITY_STEP_MPS;
            Logger.recordOutput("AutoTuner/Adjustment", "VELOCITY_UP");
        } else {
            // velocity maxed, try hood — but respect zone cap
            double cap = Math.min(zoneCap(currentDistanceM()), hoodCeiling());
            if (currentHoodDeg < cap - MIN_ANGLE_SEP_DEG) {
                currentHoodDeg = Math.min(currentHoodDeg + HOOD_STEP_DEG, cap);
                Logger.recordOutput("AutoTuner/Adjustment", "HOOD_UP");
            } else {
                // truly stuck — small velocity bump past soft cap as last resort
                currentVelocityMps += VELOCITY_STEP_MPS;
                Logger.recordOutput("AutoTuner/Adjustment", "VELOCITY_UP_FORCED");
            }
        }
        logWorkingValues();
    }

    private void adjustForLong() {
        // Prefer hood up (steeper) to bleed distance without touching velocity
        // Velocity down only if hood is already at ceiling
        double cap = Math.min(zoneCap(currentDistanceM()), hoodCeiling());
        if (currentHoodDeg < cap - MIN_ANGLE_SEP_DEG) {
            currentHoodDeg = Math.min(currentHoodDeg + HOOD_STEP_DEG, cap);
            Logger.recordOutput("AutoTuner/Adjustment", "HOOD_UP_FOR_LONG");
        } else {
            currentVelocityMps = Math.max(currentVelocityMps - VELOCITY_STEP_MPS, 10.0);
            Logger.recordOutput("AutoTuner/Adjustment", "VELOCITY_DOWN");
        }
        logWorkingValues();
    }

    // -------------------------------------------------------------------------
    // Zone cap — enforces flat arc preference by distance
    // -------------------------------------------------------------------------

    private double zoneCap(double distM) {
        if (distM < CLOSE_RANGE_THRESHOLD) return CLOSE_RANGE_MAX_DEG;
        if (distM < LONG_RANGE_THRESHOLD) return MID_RANGE_MAX_DEG;
        return LONG_RANGE_MAX_DEG;
    }

    // -------------------------------------------------------------------------
    // Monotonicity constraints from confirmed neighbors
    // -------------------------------------------------------------------------

    private double hoodCeiling() {
        double dist = currentDistanceM();
        return confirmedPoints.stream()
            .filter(p -> p.distanceM() < dist)
            .max(Comparator.comparingDouble(TunerPoint::distanceM))
            .map(p -> p.hoodDeg() - MIN_ANGLE_SEP_DEG)
            .orElse(HOOD_MAX_DEG);
    }

    private double hoodFloor() {
        double dist = currentDistanceM();
        return confirmedPoints.stream()
            .filter(p -> p.distanceM() > dist)
            .min(Comparator.comparingDouble(TunerPoint::distanceM))
            .map(p -> p.hoodDeg() + MIN_ANGLE_SEP_DEG)
            .orElse(HOOD_MIN_DEG);
    }

    // -------------------------------------------------------------------------
    // Confirm and log
    // -------------------------------------------------------------------------

    private void confirmAndLog() {
        double dist = currentDistanceM();
        TunerPoint point = new TunerPoint(dist, currentVelocityMps, currentHoodDeg);
        confirmedPoints.add(point);
        confirmedPoints.sort(Comparator.comparingDouble(TunerPoint::distanceM));
        pointConfirmedThisDistance = true;

        Logger.recordOutput("AutoTuner/ConfirmedPoints/Count", confirmedPoints.size());
        Logger.recordOutput("AutoTuner/LastConfirmed/DistanceM", dist);
        Logger.recordOutput("AutoTuner/LastConfirmed/VelocityMps", currentVelocityMps);
        Logger.recordOutput("AutoTuner/LastConfirmed/HoodDeg", currentHoodDeg);
        Logger.recordOutput("AutoTuner/Adjustment", "CONFIRMED");

        logSession();
    }

    private boolean lastPointConfirmed() {
        return pointConfirmedThisDistance;
    }

    private double currentDistanceM() {
        return ShooterStructure.distanceToTarget(AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D));
    }

    private void logWorkingValues() {
        Logger.recordOutput("AutoTuner/Working/VelocityMps", currentVelocityMps);
        Logger.recordOutput("AutoTuner/Working/HoodDeg", currentHoodDeg);
    }

    private void logSession() {
        int n = confirmedPoints.size();
        double[] distances = new double[n];
        double[] velocities = new double[n];
        double[] hoods = new double[n];

        for (int i = 0; i < n; i++) {
            distances[i] = confirmedPoints.get(i).distanceM();
            velocities[i] = confirmedPoints.get(i).velocityMps();
            hoods[i] = confirmedPoints.get(i).hoodDeg();
        }

        Logger.recordOutput("AutoTuner/Session/DistancesM", distances);
        Logger.recordOutput("AutoTuner/Session/VelocitiesMps", velocities);
        Logger.recordOutput("AutoTuner/Session/HoodDegrees", hoods);

        System.out.println("=== AutoTuner Session Results ===");
        for (int i = 0; i < n; i++) {
            System.out.printf(
                "shootingFlywheelVelocityMap.put(%.3f, %.3f);%n", distances[i], velocities[i]);
        }
        System.out.println();
        for (int i = 0; i < n; i++) {
            System.out.printf(
                "shootingHoodAngleMap.put(%.3f, Rotation2d.fromDegrees(%.1f));%n", distances[i], hoods[i]);
        }
        System.out.println("=================================");
    }
}
