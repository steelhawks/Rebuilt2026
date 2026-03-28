package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {

        boolean isConnected = false;
        Rotation2d position = new Rotation2d();
        Rotation2d velocityRadPerSec = new Rotation2d();
        double currentAmps = 0.0;
        double tempCelsius = 0.0;
        double appliedVolts = 0.0;
        double torqueCurrent = 0.0;
        double statorCurrent = 0.0;

        boolean encoderConnected = false;
        Rotation2d encoderPosition = new Rotation2d();
        Rotation2d encoderVelocity = new Rotation2d();
        double encoderVoltage = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void runOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runTurret(double output) {}

    default void stopTurret() {}

    default void setBrakeMode(boolean enabled) {}

    default void setTurretPosition(double position) {}

    default void setTurretPID(double kP, double kI, double kD) {}

    default void runTurretPivot(double setpoint, double output) {}
}
