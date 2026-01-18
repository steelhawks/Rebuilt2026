package org.steelhawks.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean connected = false;
        public double velocityRadPerSec = 0;
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double tempCelsius = 0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void runOpenLoop(double volts) {}

    default void runVelocity(double speed) {}

    default void stop() {}
}
