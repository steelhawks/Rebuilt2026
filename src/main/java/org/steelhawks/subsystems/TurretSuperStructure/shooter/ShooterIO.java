package org.steelhawks.subsystems.TurretSuperStructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {

        public boolean isConnected = false;
        public double positionRad = 0;
        public double velocityRadPerSec = 0;
        public double appliedVolts = 0;
        public double currentsAmps = 0;
        public double tempCelsius = 0;

    }

    default void updateInputs(ShooterIOInputs inputs) {

    }

    default void runOpenLoop(double percentageOutput) {

    }

    default void runSpeed(double speed) {

    }

    default void stop() {

    }
}
