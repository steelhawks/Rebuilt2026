package org.steelhawks;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.util.LoggedTunableNumber;

public class BuilderConstants {

    public static IntakeConstants IntakeConstants;

    public static class OmegaBot {
        public static final IntakeConstants INTAKE = new IntakeConstants(
                1, 2, 3,
                5.0, 0.0, 0.0,
                200.0, 0.0, 0.0,
                0.8, 2.0,
                40.0, 0.05,
                1.0);

        public static final FlywheelConstants FLYWHEEL = new FlywheelConstants(
                1,
                2,
                0.85,
                0.5,
                0.2,
                0.8,
                3,
                5,
                300,
                5,
                4,
                2/1);

        public static final TurretConstants TURRET = new TurretConstants(4, 9, 3000.0, 0.0, 100.0, 5.5, 0.0, 20.0, 30.0, 0.0, Rotation2d.fromRadians(-2.284097), Rotation2d.fromRadians(2.666059), Rotation2d.fromRotations(0.064697265625));

    }

     public static class TurretPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret kS", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class HoodPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood kS", 0.0);
        public static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood kG", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble())
                    .withKG(kG.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class FlywheelPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel kS", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKP(kP.getAsDouble())
                    .withKI(kI.getAsDouble())
                    .withKD(kD.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class ExtensionPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Extension kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/Extension kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Extension kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Extension kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Extension kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Extension kS", 0.0);
        public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/Extension kG", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble())
                    .withKG(kG.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public static class RollerPIDConstants {
        public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/Roller kP", 0.0);
        public static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/Roller kI", 0.0);
        public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/Roller kD", 0.0);
        public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/Roller kA", 0.0);
        public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/Roller kV", 0.0);
        public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/Roller kS", 0.0);

        public static void setPID(TalonFX motor) {
            var rollerSlot0Configs = new Slot0Configs();
            rollerSlot0Configs
                    .withKA(kA.getAsDouble())
                    .withKS(kS.getAsDouble())
                    .withKV(kV.getAsDouble());

            motor.getConfigurator().apply(rollerSlot0Configs);
        }
    }

    public record IntakeConstants(
            int leftId, int rightId, int intakeId,
            double kS, double kG, double kA, double kP, double kI, double kD,
            double maxVelocityMetersPerSec, double maxAccelMetersPerSecSq,
            double currentHomingThreshold,
            double intakeSpeed, double outtakeSpeed
    ) {
        public static final IntakeConstants UNSET =
                new IntakeConstants(0, 0, 0, 0,0, 0, 0, 0,0, 0, 0, 0, 0, 0);
    }

    public record FlywheelConstants(
            int leftMotorId,
            int rightMotorId,
            double idleMultiplier,
            double kP, double kI, double kD, double kS, double kV,
            double velocityToleranceRadPerSec,
            double stationaryHoodVelocityFactor,
            double flywheelRadius,
            double reduction
    ) {
        public static final FlywheelConstants UNSET =
                new FlywheelConstants(0, 0, 0, 0, 0, 0,  0, 0, 0, 0,0, 0);
    }

    public record TurretConstants(
            int turretId, int encoderId,
            double kP, double kI, double kD, double kS, double kA,
            double maxVelocityRadPerSec,
            double maxAccelerationRadPerSecSq,
            double motorReduction,
            Rotation2d minRotation,
            Rotation2d maxRotation,
            Rotation2d encoderOffset
    ) {
        public static final TurretConstants UNSET =
                new TurretConstants(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new Rotation2d(0), new Rotation2d(0),new Rotation2d(0) );
    }

}
