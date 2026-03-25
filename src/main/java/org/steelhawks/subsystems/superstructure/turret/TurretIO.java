package org.steelhawks.subsystems.superstructure.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {

        boolean isConnected = false;
        double position = 0.0;
        double velocityRadPerSec = 0.0;
        double currentAmps = 0.0;
        double tempCelsius = 0.0;
        double appliedVolts = 0.0;
        double torqueCurrent = 0.0;
        double statorCurrent = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void runOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runTurret(double position, double output, boolean isTorqueCurrent) {}

    default void stopTurret() {}

    default void setBrakeMode(boolean enabled) {}

    default void setTurretPID(double kP, double kI, double kD) {}
}
