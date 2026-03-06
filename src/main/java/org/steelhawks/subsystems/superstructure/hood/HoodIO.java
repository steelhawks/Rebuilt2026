package org.steelhawks.subsystems.superstructure.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    class HoodIOInputs {
        public double goal = 0.0;

        public boolean motorConnected = false;
        public Rotation2d motorPositionDeg = Rotation2d.fromDegrees(0.0);
        public double motorVelocityDegPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;

        public boolean cancoderConnected = false;
        public Rotation2d cancoderPositionDeg = Rotation2d.fromDegrees(0.0);
        public double cancoderVelocityDegPerSec = 0.0;
        public double cancoderAppliedVolts = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}

    default void setBrakeMode(boolean enabled) {}

    default void runHoodPosition(Rotation2d setpoint, double feedforward) {}

    default void runOpenLoop(double output, boolean isTorqueCurrent) {}

    default void setPID(double kP, double kI, double kD) {}

    default void stop() {}
}
