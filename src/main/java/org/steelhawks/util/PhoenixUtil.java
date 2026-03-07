package org.steelhawks.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
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

    public static void registerSignals(RobotConfig.CANBusList bus, BaseStatusSignal... signals) {
        if (bus.bus.isNetworkFD()) {
            var selectedBusSignals = bus.bus.equals(RobotConfig.CANBusList.kDrivetrainBus)
                ? drivetrainCanivoreSignals
                : turretCanivoreSignals;
            BaseStatusSignal[] newSignals = new BaseStatusSignal[selectedBusSignals.length + signals.length];
            System.arraycopy(selectedBusSignals, 0, newSignals, 0, selectedBusSignals.length);
            System.arraycopy(signals, 0, newSignals, selectedBusSignals.length, signals.length);
            if (bus.bus.equals(RobotConfig.CANBusList.kDrivetrainBus)) {
                drivetrainCanivoreSignals = newSignals;
            } else {
                turretCanivoreSignals = newSignals;
            }
        } else {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
            System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
            rioSignals = newSignals;
        }
    }

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
        if (canivore) {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[drivetrainCanivoreSignals.length + signals.length];
            System.arraycopy(drivetrainCanivoreSignals, 0, newSignals, 0, drivetrainCanivoreSignals.length);
            System.arraycopy(signals, 0, newSignals, drivetrainCanivoreSignals.length, signals.length);
            drivetrainCanivoreSignals = newSignals;
        } else {

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

//    @Override
//    public void periodic() {
//        refreshAll();
//        LoopTimeUtil.record("PhoenixUtil");
//    }
}
