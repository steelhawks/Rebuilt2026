package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    class TurretIOInputs {
        public boolean connected = false;
        public Rotation2d positionRad = new Rotation2d();
        public Rotation2d velocityRadPerSec = new Rotation2d();
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setBrakeMode(boolean enabled) {}

    default void runPivot(double setpoint, double feedforward) {}

    default void runOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runPercentOutput(double output) {}

    default void setPID(double kp, double ki, double kd) {}

    default void stop() {}
}
