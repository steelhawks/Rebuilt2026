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

    /** Signals for Phoenix Pro Timesync */
    private static final double TURRET_TIMESYNC_TIMEOUT_S = 1.0 / 100.0;
    private static BaseStatusSignal[] turretTimesyncSignals = new BaseStatusSignal[0];

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
        if (bus.isNetworkFD()) {
            var selectedBusSignals = bus.equals(RobotConfig.CANBusList.kDrivetrainBus)
                ? drivetrainCanivoreSignals
                : turretCanivoreSignals;
            BaseStatusSignal[] newSignals = new BaseStatusSignal[selectedBusSignals.length + signals.length];
            System.arraycopy(selectedBusSignals, 0, newSignals, 0, selectedBusSignals.length);
            System.arraycopy(signals, 0, newSignals, selectedBusSignals.length, signals.length);
            if (bus.equals(RobotConfig.CANBusList.kDrivetrainBus)) {
                drivetrainCanivoreSignals = newSignals;
            } else if (bus.equals(RobotConfig.CANBusList.kTurretBus)) {
                turretCanivoreSignals = newSignals;
            } else {
                DriverStation.reportWarning("Unknown CANivore bus: " + bus.getName(), false);
                throw new RuntimeException("Unknown CANivore bus: " + bus.getName());
            }
        } else {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
            System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
            rioSignals = newSignals;
        }
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

    /**
     * Register signals for timesync via waitForAll.
     */
    public static void registerTimesyncedSignals(CANBus bus, BaseStatusSignal... signals) {
        if (!bus.isNetworkFD()) {
            DriverStation.reportWarning("RIO bus does not support timesync, registering as normal signals.", false);
            registerSignals(bus, signals);
            return;
        }
        if (bus.equals(RobotConfig.CANBusList.kTurretBus)) {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[turretTimesyncSignals.length + signals.length];
            System.arraycopy(turretTimesyncSignals, 0, newSignals, 0, turretTimesyncSignals.length);
            System.arraycopy(signals, 0, newSignals, turretTimesyncSignals.length, signals.length);
            turretTimesyncSignals = newSignals;
        } else {
            DriverStation.reportWarning("Unknown CANivore bus: " + bus.getName(), false);
            throw new RuntimeException("Unknown CANivore bus: " + bus.getName());
        }
    }

    /**
     * Block until all timesync signals arrive with a synchronized timestamp.
     */
    public static void waitForAll(CANBus bus, double timeout) {
       if (bus.equals(RobotConfig.CANBusList.kTurretBus)) {
            if (turretTimesyncSignals.length > 0) {
                BaseStatusSignal.waitForAll(timeout, turretTimesyncSignals);
            }
        } else {
            DriverStation.reportWarning("Unknown CANivore bus: " + bus.getName(), false);
            throw new RuntimeException("Unknown CANivore bus: " + bus.getName());
        }
    }
}
