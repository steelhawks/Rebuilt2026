package org.steelhawks.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean motor1Connected = false;
        public double motor1VelocityRadPerSec = 0;
        public double motor1AppliedVolts = 0;
        public double motor1Current = 0;
        public double motor1Temp = 0;
        public double velocityGoal = 0;

        public boolean motor2Connected = false;
        public double motor2VelocityRadPerSec = 0;
        public double motor2AppliedVolts = 0;
        public double motor2Current = 0;
        public double motor2Temp = 0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void runOpenLoop(double volts) {}

    default void runVelocity(double rps, double feedforward) {}

    default void setPID(double kP, double kI, double kD) {}

    default void stop() {}
}
