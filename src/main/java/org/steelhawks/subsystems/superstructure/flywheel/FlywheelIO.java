package org.steelhawks.subsystems.superstructure.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    class FlywheelIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void runFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {}

    default void runProfiledFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {}

    default void runFlywheelOpenLoop(double output, boolean isTorqueCurrent) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setProfile(double accel, double jerk) {}

    default void stop() {}
}
