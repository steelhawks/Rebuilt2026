package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
     class IntakeIOInputs {
        // right electrical data
        public boolean rightConnected = false;
        public double rightExtensionPosition = 0.0;
        public double rightExtensionVelocity = 0.0;
        public double rightExtensionCurrentAmps = 0.0;
        public double rightExtensionTempCelsius = 0.0;
        public double rightExtensionAppliedVolts = 0.0;
        public double rightExtensionTorqueCurrent = 0.0;
        public double rightExtensionStatorCurrent = 0.0;

        // left electrical data
        public boolean leftConnected = false;
        public double leftExtensionPosition = 0.0;
        public double leftExtensionVelocity = 0.0;
        public double leftExtensionCurrentAmps = 0.0;
        public double leftExtensionTempCelsius = 0.0;
        public double leftExtensionAppliedVolts = 0.0;
        public double leftExtensionTorqueCurrent = 0.0;
        public double leftExtensionStatorCurrent = 0.0;

        // intake electrical data
        public boolean rollerConnected = false;
        public Rotation2d rollerPosition = new Rotation2d();
        public double rollerVelocity = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerTorqueCurrent = 0.0;
        public double rollerStatorCurrent = 0.0;

        // states
        public boolean isIntaking = false;
        public boolean isEjecting = false;
        public boolean isExtended = false;
        public boolean isRetracted = true;

    }

    // update all data
    default void updateInputs(IntakeIOInputs inputs) {}

    // velocityVoltage most likely
    default void setExtensionVoltage(double volts) {}

    // feedforward
    default void setExtensionVelocity(double velocityPerSec, double ffOutput) {}

    // pid and feedforward
    default void setExtensionPosition(double position, double ffOutput) {}

    // stop only extension
    default void stopExtension() {}


    // set the brake type
    default void setExtensionBrakeMode(boolean enabled) {}

    default void resetExtension(double position) {}

    // velocityVoltage most likely
    default void setRollerVoltage(double volts) {}

    // feedforward
    default void setRollerVelocity(double velocityPerSec, double ffOutput) {}


    // stop only extension
    default void stopRoller() {}


    // set the brake type
    default void setRollerBrakeMode(boolean enabled) {}

    default void stopAll() {
        stopExtension();
        stopRoller();
    }

    default double getExtensionSetpoint() {
        return 0.0;
    }

    default double getExtensionVelocitySetpoint() {
        return 0.0;
    }

    default double getRollerVelocitySetpoint() {
        return 0.0;
    }

    default void setExtensionPID(double kP, double kI, double kD) {}

    default void setRollerPID(double kP, double kI, double kD) {}

}
