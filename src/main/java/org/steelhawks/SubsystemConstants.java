package org.steelhawks;


public class SubsystemConstants {
    public record FlywheelConstants(
        int leftMotorId,
        int rightMotorId,
        double flywheelRadius,
        double reduction,
        double idleMultiplier,
        double kP, double kI, double kD, double kS, double kV,
        double samplingTimeoutDuration,
        double timeoutAvgMinSamples,
        int sampleCounts,
        double stationaryHoodVelocityFactor
    ) {
        public static final FlywheelConstants UNSET = new FlywheelConstants(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record TurretConstants(
        int turretId,
        double kP, double kI, double kD, double kS, double kA,
        double maxVelocityRadPerSec,
        double maxAccelerationRadPerSecSq,
        double manualIncrement
    ) {
        public static final TurretConstants UNSET = new TurretConstants(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record HoodConstants(
        int motorId, int cancoderId,
        double kP, double kI, double kD, double kS, double kG, double kV
    ) {
        public static final HoodConstants UNSET = new HoodConstants(0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record IntakeConstants(
        int leftId, int rightId, int driveId,
        double kS, double kG, double kA, double kP, double kI, double kD,
        double maxVelocityMetersPerSec, double maxAccelMetersPerSecSq,
        double currentHomingThreshold,
        double velocityStallingThreshold,
        double intakeSpeed, double outtakeSpeed
    ) { }
    public static final class AlphaBot {
        public static final IntakeConstants INTAKE_ALPHA = new IntakeConstants(
            60, 61, 62,
            0, 0, 0,
            5, 0, 0,
            0.05, 0.08,
            60, 0.003,
            1, 1
        );
    }
}
