package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    class ClimbIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

    default void updateInputs(ClimbIOInputs inputs) {}

    default void setBrakeMode(boolean enabled) {}

    default void runPosition(Rotation2d setpoint, double feedforward) {}

    default void runOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runPercentOut(double output) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kA, double kV) {}

    default void setPosition(double position) {}

    default void stop() {}
}
