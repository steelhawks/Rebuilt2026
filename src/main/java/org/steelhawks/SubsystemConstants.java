package org.steelhawks;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import java.util.OptionalInt;

/**
 * This class contains constants that are selected by RobotConfig and passed to the relevant subsystem upon init.
 *
 * @author Adam Aptowitz
 */
public class SubsystemConstants {
    public record LUTConstants (
        double minShootDistance, double maxShootDistance,
        double minFerryDistance, double maxFerryDistance,
        double[][] shootingTimeOfFlightMap,
        double[][] shootingFlywheelVelocityMap,
        double[][] shootingHoodAngleMap,
        double[][] ferryTimeOfFlightMap,
        double[][] ferryFlywheelVelocityMap,
        double[][] ferryHoodAngleMap
    ) {
        public static final LUTConstants UNSET =
            new LUTConstants(0.0,  Double.MAX_VALUE, 0.0, Double.MAX_VALUE, null, null, null, null, null, null);
    }


    public record FlywheelConstants(
        int leftMotorId,
        int rightMotorId,
        double idleMultiplier,
        double kP, double kI, double kD, double kS, double kV,
        double velocityToleranceRadPerSec,
        double samplingTimeoutDuration,
        double timeoutAvgMinSamples,
        double stationaryHoodVelocityFactor,
        double flywheelRadius,
        double reduction
    ) {
        public static final FlywheelConstants UNSET =
            new FlywheelConstants(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record TurretConstants(
        int turretId, int encoderId,
        double kP, double kI, double kD, double kS, double kA,
        double maxVelocityRadPerSec,
        double maxAccelerationRadPerSecSq,
        double manualIncrement,
        double currentHomingThreshold,
        double motorReduction,
        Rotation2d minRotation,
        Rotation2d maxRotation,
        Rotation2d encoderOffset
    ) {
        public static final TurretConstants UNSET =
            new TurretConstants(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new Rotation2d(), new Rotation2d(), new Rotation2d());
    }

    public record HoodConstants(
        int motorId, int cancoderId,
        double kP, double kI, double kD,
        double kS, double kG, double kA,
        double maxVelocity, double maxAcceleration,
        double reduction,
        Rotation2d minAngle, Rotation2d maxAngle,
        Rotation2d magOffset,
        double tolerance
    ) {
        public static final HoodConstants UNSET =
            new HoodConstants(0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, new Rotation2d(), new Rotation2d() , new Rotation2d(), 0);
    }

    public record IntakeConstants(
        int leftId, int rightId, int intakeId,
        double kS, double kG, double kA, double kP, double kI, double kD,
        double maxVelocityMetersPerSec, double maxAccelMetersPerSecSq,
        double currentHomingThreshold,
        double velocityStallingThreshold,
        double intakeSpeed, double outtakeSpeed,
        double positionTwistingThreshold
    ) {
        public static final IntakeConstants UNSET =
            new IntakeConstants(0, 0, 0, 0,0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0);
    }

    public record IndexerConstants(
        int spindexerMotor1Id, int feederId,
        double indexerJamCurrent, double feederJamCurrent,
        OptionalInt spindexerMotor2Id,
        OptionalInt beamId
    ) {
        public static final IndexerConstants UNSET =
            new IndexerConstants(0, 0, 0, 0, OptionalInt.empty(), OptionalInt.empty());
    }

    public static final class OmegaBot {
        public static final LUTConstants LUT =
            new LUTConstants(
                0.0, Double.MAX_VALUE,
                0.0, Double.MAX_VALUE,
                new double[][]{ // Shooting ToF
                    {1.210, 1},
                    {1.870, 1},
                    {2.290, 1},
                    {2.920, 1},
                    {3.590, 1}
                },
                new double[][]{ // Shooting Velocity
                    {1.210, 10.0},
                    {1.870, 10.5},
                    {2.290, 10.7},
                    {2.920, 11},
                    {3.590, 12}
                },
                new double[][]{ // Shooting Hood Angle
                    {1.210, 80.0},
                    {1.870, 75.0},
                    {2.290, 70.0},
                    {2.920, 65.0},
                    {3.590, 63.0}
                },
                new double[][]{ // Ferry ToF
                    {1.75, 14.8},
                    {2.12, 15.0},
                    {3.16, 18.8}
                },
                new double[][]{ // Ferry Velocity
                    {1.75, 14.8},
                    {2.12, 15.0},
                    {3.16, 18.8}
                },
                new double[][]{ // Ferry Hood Angle
                    {1.75, 14.8},
                    {2.12, 15.0},
                    {3.16, 18.8}
                });

        public static final IntakeConstants INTAKE =
            new IntakeConstants(
                1, 2, 3,
                5.0, 0.0, 0.0,
                200.0, 0.0, 0.0,
                3.0, 5.0,
                35.0, 0.05,
                0.6, -1.0,
                0.8);

        public static final FlywheelConstants FLYWHEEL =
            new FlywheelConstants(
                5, 6, 0.85, 10.0, 0.0, 0.0, 6.94627, 0.02, 5.0, 1.0, 50, 1.68, Units.inchesToMeters(2.0), (1.0 / 1.0));

//    ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44000 ﻿﻿ DS Return Key Disable ﻿﻿ Driver Station ﻿﻿﻿
//            ﻿﻿﻿﻿﻿﻿ ********** Flywheel FF Characterization Results ********** ﻿
//            ﻿﻿﻿﻿﻿﻿ 	kS: 7.44627 ﻿
//            ﻿﻿﻿﻿﻿﻿ 	kV: 0.04831

        public static final TurretConstants TURRET =
            new TurretConstants(
                4, 9, 5000.0, 0.0, 200.0, 13.6, 0.0, 20.0, 30.0, 0.0, 0.0, (18.0 / 18.0) * (46.0 / 18.0) * (96.0 / 12.0), Rotation2d.fromRadians(-2.485049), Rotation2d.fromRadians(3.402369), Rotation2d.fromRotations(-0.227294921875));
        // Turret Characterization Results
        // kS: 13.18095
        // kV: 1.99614

        // kS: 15.17513
        // kv: 1.71165

        // kS: 10.67311
        // kv: 2.54928 omg cheesy poofs

        // kS: 13.57013
        // kv: 1.92354


        // Turret Characterization Results
        // kS: 13.18095
        // kV: 1.99614

        // kS: 15.17513
        // kv: 1.71165

        // kS: 10.67311
        // kv: 2.54928 omg cheesy poofs

        // kS: 13.57013
        // kv: 1.92354

        public static final HoodConstants HOOD =
            new HoodConstants(
                    7, 8, 200.0, 0.0, 0.0, 0.42170, 0.0, 0.0,  10.0, 12.0, 81.95, Rotation2d.fromDegrees(40.0), Rotation2d.fromDegrees(80.0), Rotation2d.fromRotations(0.382080078125).plus(Rotation2d.fromDegrees(80.0)), 4.0);

//        ﻿﻿﻿﻿﻿﻿ ********** Hood FF Characterization Results ********** ﻿
//            ﻿﻿﻿﻿﻿﻿ 	kS: -0.42170 ﻿
//            ﻿﻿﻿﻿﻿﻿ 	kV: 0.02385 ﻿


        public static final IndexerConstants INDEXER =
            new IndexerConstants(
                1, 3, 40.0, 50.0, OptionalInt.of(2), OptionalInt.of(0));
    }

    public static final class AlphaBot {
        public static final LUTConstants LUT =
            new LUTConstants(
                0.0, Double.MAX_VALUE,
                0.0, Double.MAX_VALUE,
                null,
                new double[][]{
                    {1.75, 14.8},
                    {2.12, 15.0},
                    {3.16, 18.8}
                },
                null,
                null,
                null,
                null);

        public static final IntakeConstants INTAKE =
            new IntakeConstants(
                60, 61, 62,
                0, 0, 0,
                5, 0, 0,
                0.05, 0.08,
                60, 0.03,
                0.6, 0.6,
                0.05
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
                2,
                Units.inchesToMeters(2),
                1.0 / 2.0
            );

        public static final TurretConstants TURRET =
            new TurretConstants(
                1, 0,
                1100, 0, 90,
                3.0, 0,
                10, 20,
                0.1,
                5,
                (200.0 / 20.0),
                new Rotation2d((-Math.PI / 2) - (Math.PI / 60)),
                new Rotation2d(Math.PI + (Math.PI / 60)),
                new Rotation2d()
            );

        public static final IndexerConstants INDEXER =
            new IndexerConstants(
                45, 26,
                40, 40,
                OptionalInt.empty(),
                OptionalInt.empty()
            );
    }

    public static final class SimBot {
        public static final IntakeConstants INTAKE =
            new IntakeConstants(
                60, 61, 62,
                0, 0, 0,
                5, 0, 0,
                0.05, 0.08,
                60, 0.03,
                1, 1,
                0.05
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
                2,
                Units.inchesToMeters(2),
                1.0 / 2.0
            );

        public static final TurretConstants TURRET =
            new TurretConstants(
                1, 0,
                200, 0, 7,
                0.2, 0,
                10, 20,
                0.1,
                40,
                (200.0 / 20.0),
                new Rotation2d((-Math.PI / 2) - (Math.PI / 60)),
                new Rotation2d(Math.PI + (Math.PI / 60)),
                new Rotation2d()
            );

        // hood constants
        // copied from previous constants file for kG calculation, since some math needs to be done while creating the record
        private static final double M = Units.lbsToKilograms(0.0);
        private static final double G = 9.81;
        private static final double R = Units.inchesToMeters(0.0); // dist from pivot point to CoM
        private static final double kT = DCMotor.getKrakenX44Foc(1).KtNMPerAmp;
        private static final double HOOD_REDUCTION = 4.357 / 1.0;
        private static final  Rotation2d minAngle = Rotation2d.fromDegrees(40.0);

        public static final HoodConstants HOOD = new HoodConstants(
            0, 0,
            0, 0, 0,
            0,
            (M * G * R) / (kT * HOOD_REDUCTION),
            0,
            10.0, 20.0,
            HOOD_REDUCTION,
            // min angle is full extension, max angle is home
            minAngle, Rotation2d.fromDegrees(80.0),
            Rotation2d.fromRotations(0.0).plus(minAngle),
            0.02
        );

        public static final IndexerConstants INDEXER =
            new IndexerConstants(
                45, 26,
                40, 40,
                OptionalInt.empty(),
                OptionalInt.empty()
            );
    }
}
