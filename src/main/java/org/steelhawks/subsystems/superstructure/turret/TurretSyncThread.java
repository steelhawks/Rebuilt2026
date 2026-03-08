package org.steelhawks.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.RobotController;
import org.steelhawks.RobotConfig;
import org.steelhawks.util.PhoenixUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

public class TurretSyncThread extends Thread {

    public static final double TURRET_SYNC_FREQUENCY = 100.0;

    public final Lock turretLock = new ReentrantLock();

    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static TurretSyncThread instance = null;

    public static TurretSyncThread getInstance() {
        if (instance == null) {
            instance = new TurretSyncThread();
        }
        return instance;
    }

    private TurretSyncThread() {
        setName("TurretSyncThread");
        setDaemon(true);
    }

    @Override
    public void start() {
        if (!timestampQueues.isEmpty()) {
            super.start();
        }
    }

    /**
     * Registers a generic signal (non-Phoenix) to be read from the thread.
     */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        turretLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            turretLock.unlock();
        }
        return queue;
    }

    /**
     * Returns a new queue that returns timestamp values for each sample.
     */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        turretLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            turretLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            if (RobotConfig.CANBusList.kTurretBus.isNetworkFD()) {
                PhoenixUtil.waitForAll(RobotConfig.CANBusList.kTurretBus, 1.0 / TURRET_SYNC_FREQUENCY);
            } else {
                try {
                    Thread.sleep((long) (1000.0 / TURRET_SYNC_FREQUENCY));
                    PhoenixUtil.refreshAll();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            turretLock.lock();
            try {
                double timestamp = RobotController.getFPGATime() / 1e6;

                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            } finally {
                turretLock.unlock();
            }
        }
    }
}