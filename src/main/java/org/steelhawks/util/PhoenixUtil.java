package org.steelhawks.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.steelhawks.RobotConfig;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public final class PhoenixUtil {

    /** Signals for synchronized refresh. */
    private static BaseStatusSignal[] drivetrainCanivoreSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] turretCanivoreSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    private PhoenixUtil() {
        throw new InstantiationError("PhoenixUtil is a utility class and cannot be instantiated.");
    }

    /**
     * Attempts to run the command until no error is produced.
     */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    /**
     * Used for MapleSim simulation for the Gyro and Odometry
     */
    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp()
                - 0.02
                + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }

    public static void registerSignals(CANBus bus, BaseStatusSignal... signals) {
        if (bus.isNetworkFD() && bus.equals(RobotConfig.CANBusList.kDrivetrainBus)) {
            drivetrainCanivoreSignals = append(drivetrainCanivoreSignals, signals);
        } else if (bus.isNetworkFD() && bus.equals(RobotConfig.CANBusList.kTurretBus)) {
            turretCanivoreSignals = append(turretCanivoreSignals, signals);
        } else {
            // Falls here for the RIO bus and for any bus we don't specifically recognize.
            // Some simulation/CI backends report the RIO bus as CAN FD, which previously
            // matched neither known CANivore bus and threw, killing robot construction.
            // Register those signals on the RIO bucket instead of crashing.
            if (bus.isNetworkFD()) {
                DriverStation.reportWarning(
                    "Unrecognized CAN FD bus '" + bus.getName()
                        + "'; registering its signals on the RIO bus.", false);
            }
            rioSignals = append(rioSignals, signals);
        }
    }

    private static BaseStatusSignal[] append(BaseStatusSignal[] existing, BaseStatusSignal[] added) {
        BaseStatusSignal[] combined = new BaseStatusSignal[existing.length + added.length];
        System.arraycopy(existing, 0, combined, 0, existing.length);
        System.arraycopy(added, 0, combined, existing.length, added.length);
        return combined;
    }

    /** Refresh all registered signals. */
    public static void refreshAll() {
        if (drivetrainCanivoreSignals.length > 0) {
            BaseStatusSignal.refreshAll(drivetrainCanivoreSignals);
        }
        if (turretCanivoreSignals.length > 0) {
            BaseStatusSignal.refreshAll(turretCanivoreSignals);
        }
        if (rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
        }
    }
}
