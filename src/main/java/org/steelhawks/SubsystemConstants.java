package org.steelhawks;

/**
 * This class contains constants that are selected by RobotConfig and passed to the relevant subsystem upon init.
 * Note that these records mostly consist of tunable values. For physical constants, refer to the Constants class in the subsystem folder.
 *
 * @author Adam Aptowitz
 */
public class SubsystemConstants {
    public record FlywheelConstants(
        int leftMotorId,
        int rightMotorId,
        double idleMultiplier,
        double kP, double kI, double kD, double kS, double kV,
        double velocityToleranceRadPerSec,
        double samplingTimeoutDuration,
        double timeoutAvgMinSamples,
        double stationaryHoodVelocityFactor
    ) {
        public static final FlywheelConstants UNSET =
            new FlywheelConstants(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record TurretConstants(
        int turretId,
        double kP, double kI, double kD, double kS, double kA,
        double maxVelocityRadPerSec,
        double maxAccelerationRadPerSecSq,
        double manualIncrement,
        double currentHomingThreshold
    ) {
        public static final TurretConstants UNSET =
            new TurretConstants(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record HoodConstants(
        int motorId, int cancoderId,
        double kP, double kI, double kD, double kS, double kG, double kA
    ) {
        public static final HoodConstants UNSET =
            new HoodConstants(0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record IntakeConstants(
        int leftId, int rightId, int driveId,
        double kS, double kG, double kA, double kP, double kI, double kD,
        double maxVelocityMetersPerSec, double maxAccelMetersPerSecSq,
        double currentHomingThreshold,
        double velocityStallingThreshold,
        double intakeSpeed, double outtakeSpeed
    ) {
        public static final IntakeConstants UNSET =
            new IntakeConstants(0, 0, 0, 0,0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0);
    }

    public record IndexerConstants(
        int indexerId, int feederId,
        double indexerJamCurrent, double feederJamCurrent
    ) {
        public static final IndexerConstants UNSET =
            new IndexerConstants(0, 0, 0, 0);
    }

    public static final class AlphaBot {
        public static final IntakeConstants INTAKE =
            new IntakeConstants(
                60, 61, 62,
                0, 0, 0,
                5, 0, 0,
                0.05, 0.08,
                60, 0.003,
                0.6, 0.6
            );

        public static final FlywheelConstants FLYWHEEL =
            new FlywheelConstants(
                2, 3,
                0.5,
                0.2, 0, 0,
                0.42995, 0.0090372 * 0.9, // 10% reduction from sysid value
                20,
                2,
                10,
                2
            );

        public static final TurretConstants TURRET =
            new TurretConstants(
                1,
                1000, 0, 70,
                2, 0,
                10, 20,
                0.1,
                40
            );

        public static final IndexerConstants INDEXER =
            new IndexerConstants(
                45, 26,
                40, 40
            );
    }

    public static final class OmegaBot {
        public static final IntakeConstants INTAKE = IntakeConstants.UNSET;
        public static final FlywheelConstants FLYWHEEL = FlywheelConstants.UNSET;
        public static final TurretConstants TURRET = TurretConstants.UNSET;
        public static final HoodConstants HOOD =
            new HoodConstants(
                0, 0,
                50, 0, 5,
                3, 0, 0
            );
        public static final IndexerConstants INDEXER = IndexerConstants.UNSET;
    }

    public static final class SimBot {
        public static final IntakeConstants INTAKE =
            new IntakeConstants(
                60, 61, 62,
                0, 0, 0,
                5, 0, 0,
                0.05, 0.08,
                60, 0.003,
                1, 1
            );

        public static final FlywheelConstants FLYWHEEL =
            new FlywheelConstants(
                2, 3,
                0.5,
                0.2, 0, 0,
                0.42995, 0.0090372 * 0.9, // 10% reduction from sysid value
                20,
                2,
                10,
                2
            );

        public static final TurretConstants TURRET =
            new TurretConstants(
                1,
                200, 0, 7,
                0.2, 0,
                10, 20,
                0.1,
                40
            );

        public static final HoodConstants HOOD = HoodConstants.UNSET;

        public static final IndexerConstants INDEXER =
            new IndexerConstants(
                45, 26,
                40, 40
            );
    }
}
