package org.steelhawks.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.steelhawks.RobotConfig;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;

public final class PhoenixUtil extends VirtualSubsystem {

    private static double lastStateChangeTime = Timer.getFPGATimestamp();
    private static boolean lastSwitchState = false;

    /** Signals for synchronized refresh. */
    private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] turretSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    private PhoenixUtil() {
        throw new InstantiationError("PhoenixUtil is a utility class and cannot be instantiated.");
    }

    /**
     * Attempts to check if a DigitalInput is connected by checking if the state has changed.
     *
     * @param timeoutSecs the time in seconds to wait for a state change before assuming the DigitalInput is disconnected
     * @param mLimitSwitch the DigitalInput to check
     */
    public static boolean digitalInputAlive(int timeoutSecs, DigitalInput mLimitSwitch) {
        boolean currentState = mLimitSwitch.get();
        boolean changed = (currentState != lastSwitchState);

        if (changed) {
            lastSwitchState = currentState;
            lastStateChangeTime = Timer.getFPGATimestamp();
        }

        // if no change assume disconnected
        return changed || (Timer.getFPGATimestamp() - lastStateChangeTime < timeoutSecs);
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

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(CANBus bus, BaseStatusSignal... signals) {
        if (bus.isNetworkFD()) {
            var busSignals = bus.equals(RobotConfig.CANBusList.kDrivetrainBus) ? canivoreSignals : turretSignals;
            BaseStatusSignal[] newSignals = new BaseStatusSignal[busSignals.length + busSignals.length];
            System.arraycopy(busSignals, 0, newSignals, 0, busSignals.length);
            System.arraycopy(signals, 0, newSignals, busSignals.length, signals.length);
            if (bus.equals(RobotConfig.CANBusList.kDrivetrainBus)) {
                canivoreSignals = newSignals;
                BaseStatusSignal.setUpdateFrequencyForAll(1000, canivoreSignals);
            } else if (bus.equals(RobotConfig.CANBusList.kTurretBus)) {
                turretSignals = newSignals;
                BaseStatusSignal.setUpdateFrequencyForAll(1000, turretSignals);
            }
        } else {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
            System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
            System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
            rioSignals = newSignals;
        }
    }

    /** Refresh all registered signals. */
    public static boolean refreshAll() {
        if (canivoreSignals.length > 0) {
            BaseStatusSignal.refreshAll(canivoreSignals);
            return true;
        }
        if (rioSignals.length > 0) {
            BaseStatusSignal.refreshAll(rioSignals);
            return true;
        }
        if (turretSignals.length > 0) {
            BaseStatusSignal.refreshAll(turretSignals);
            return true;
        }
        return false;
    }

    /** Wait until all for synchronization */
    public static void waitForAll() {
        if (canivoreSignals.length > 0) {
            BaseStatusSignal.waitForAll(0.1, canivoreSignals);
        }
        if (rioSignals.length > 0) {
            BaseStatusSignal.waitForAll(0.1, rioSignals);
        }
        if (turretSignals.length > 0) {
            BaseStatusSignal.waitForAll(0.1, turretSignals);
        }
    }


    @Override
    public void periodic() {
        refreshAll();
        LoopTimeUtil.record("PhoenixUtil");
    }
}
