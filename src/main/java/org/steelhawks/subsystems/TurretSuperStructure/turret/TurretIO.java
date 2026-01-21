package org.steelhawks.subsystems.TurretSuperStructure.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    class TurretIOInputs {
        public boolean isConnected = false;
        public double amountTurned = 0.0;
        public double appliedVoltage = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void turnTurret(TurretIOInputs inputs, double percentageOutput) {}

    default void stop() {}

}
