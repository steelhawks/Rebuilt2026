package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public double goal = 0;

        public boolean leftConnected = false;
        public double leftPositionMeters = 0.0;
        public double leftVelocityMetersPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTorqueCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;

        public boolean rightConnected = false;
        public double rightPositionMeters = 0.0;
        public double rightVelocityMetersPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTorqueCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;

        public boolean intakeConnected = false;
        public Rotation2d intakePositionRad = new Rotation2d();
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeSupplyCurrentAmps = 0.0;
        public double intakeTorqueCurrentAmps = 0.0;
        public double intakeTempCelsius = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setBrakeMode(boolean enabled) {}

    default void runRackPositionBoth(double positionMeters, double leftFF, double rightFF) {}

    default void runRackPosition(double positionMeters, double feedforward) {}

    default void runRackOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runRackOpenLoopBoth(double leftOutput, double rightOutput, boolean isTorqueCurrent) {}

    default void runRackPercentOut(double output) {}

    default void setRackPID(double kP, double kI, double kD) {}

    default void setPosition(double meters) {}

    default void runIntake(double output) {}

    default void stopRack() {}

    default void stopIntake() {}
}