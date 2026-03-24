package org.steelhawks.subsystems.superstructure.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    class FlywheelIOInputs {

        boolean isConnected = false;
        double position = 0.0;
        double velocityRadPerSec = 0.0;
        double currentAmps = 0.0;
        double tempCelsius = 0.0;
        double appliedVolts = 0.0;
        double torqueCurrent = 0.0;
        double statorCurrent = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void runOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runFlywheel(double position, double ffoutput, boolean isTorqueCurrent) {}

    default void stopFlywheel() {}

    default void setFlywheelPID(double kP, double kI, double kD) {}

}
