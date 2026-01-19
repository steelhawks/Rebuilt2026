package org.steelhawks.subsystems.shooter;

public interface ShooterIO {
    class ShooterIOInputs {
        public boolean shouldRunProfile = false;
        public double goal = 0;

        public boolean masterMotorConnected = false;
        public boolean followerMotorConnected = false;

        public double masterMotorPositionRad = 0;
        public double masterMotorVelocityRPM = 0;
        public double masterMotorAppliedVolts = 0;
        public double masterMotorCurrentAmps = 0;
        public double masterMotorTemperatureCelsius = 0;

//        public double followerMotorPositionRad = 0;
//        public double followerMotorVelocityRPM = 0;
//        public double followerMotorAppliedVolts = 0;
//        public double followerMotorCurrentAmps = 0;
//        public double followerMotorTemperatureCelsius = 0;
    }
    default void updateInputs(ShooterIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runSpeed(double speed) {}

    default void zeroEncoders() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}

    default void runShooter(double shooterVolts) {}

    default void stop() {}
}
