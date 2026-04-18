package org.steelhawks.subsystems.superstructure.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    class FlywheelIOInputs {
        public boolean leftConnected = false;
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTorqueCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;


        public boolean rightConnected = false;
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTorqueCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    default void runFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {}

    default void runFlywheelOpenLoop(double output, boolean isTorqueCurrent) {}

    default void setPID(double kP, double kI, double kD) {}

    default void stop() {}
}
