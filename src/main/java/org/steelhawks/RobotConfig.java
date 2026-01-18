package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import org.steelhawks.Constants.*;
import org.steelhawks.generated.*;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.shooter.Shooter;
import org.steelhawks.subsystems.shooter.ShooterIO;
import org.steelhawks.subsystems.shooter.ShooterIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.Vision.VisionConsumer;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;

import java.util.Objects;
import java.util.Optional;

public class RobotConfig {
    // Feature flags
    public final boolean hasLEDMatrix;
    public final boolean hasLEDStrip;
    public final boolean hasVision;
    public final boolean hasObjectVision;
    public final boolean hasAutos;
    public final boolean hasShooter;

    // Subsystem factory
    private final SubsystemFactory factory;

    private RobotConfig(Builder builder) {
        this.hasLEDMatrix = builder.hasLEDMatrix;
        this.hasLEDStrip = builder.hasLEDStrip;
        this.hasVision = builder.hasVision;
        this.hasObjectVision = builder.hasObjectVision;
        this.hasAutos = builder.hasAutos;
        this.hasShooter = builder.hasShooter;
        this.factory = Objects.requireNonNull(builder.factory, "Factory cannot be null");
    }

    // Factory methods for creating subsystems

    public Swerve createSwerve() {
        return factory.createSwerve();
    }

    public Optional<LEDMatrix> createLEDMatrix() {
        if (!hasLEDMatrix) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createLEDMatrix());
    }

    public Optional<LEDStrip> createLEDStrip() {
        if (!hasLEDStrip) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createLEDStrip());
    }

    public Optional<Vision> createVision(VisionConsumer poseConsumer) {
        if (!hasVision) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createVision(poseConsumer));
    }

    public Optional<ObjectVision> createObjectVision() {
        if (!hasObjectVision) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createObjectVision());
    }

    public Optional<Shooter> createShooter() {
        if(!hasShooter) {
            return  Optional.empty();
        }

        return Optional.ofNullable(factory.createShooter());
    }

    public static RobotConfig getConfig() {
        if (Constants.getMode() == Mode.REPLAY) {
            return getReplayConfig();
        }

        return switch (Constants.getRobot()) {
            case OMEGABOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(true)
                .withAutos(true)
                .withFactory(new OmegaBotFactory())
                .build();

            case ALPHABOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(false)
                .withAutos(true)
                .withFactory(new AlphaBotFactory()).withShooter(true)
                .build();

            case LAST_YEAR -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(false)
                .withAutos(true)
                .withFactory(new LastYearFactory())
                .build();

            case SIMBOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(true)
                .withAutos(true)
                .withFactory(new SimBotFactory()).withShooter(true)
                .build();
        };
    }

    private static RobotConfig getReplayConfig() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, ALPHABOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(false)
                .withAutos(true)
                .withFactory(new ReplayFactory()).withShooter(true)
                .build();

            default -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(false)
                .withAutos(true)
                .withFactory(new ReplayFactory()).withShooter(true)
                .build();
        };
    }

    // Builder pattern
    public static class Builder {
        private boolean hasLEDMatrix = false;
        private boolean hasLEDStrip = false;
        private boolean hasVision = false;
        private boolean hasObjectVision = false;
        private boolean hasAutos = false;
        private boolean hasShooter = false;
        private SubsystemFactory factory = null;

        public Builder withLEDMatrix(boolean enabled) {
            this.hasLEDMatrix = enabled;
            return this;
        }

        public Builder withLEDStrip(boolean enabled) {
            this.hasLEDStrip = enabled;
            return this;
        }

        public Builder withVision(boolean enabled) {
            this.hasVision = enabled;
            return this;
        }

        public Builder withObjectVision(boolean enabled) {
            this.hasObjectVision = enabled;
            return this;
        }

        public Builder withAutos(boolean enabled) {
            this.hasAutos = enabled;
            return this;
        }

        public Builder withShooter(boolean enabled) {
            this.hasShooter = enabled;
            return this;
        }

        public Builder withFactory(SubsystemFactory factory) {
            this.factory = factory;
            return this;
        }

        public RobotConfig build() {
            if (factory == null) {
                throw new IllegalStateException("Factory must be set");
            }
            return new RobotConfig(this);
        }
    }

    // Base CANBus class
    public static class CANBus {
        public final String name;
        public final com.ctre.phoenix6.CANBus bus;

        public CANBus(String name) {
            this.name = Objects.requireNonNull(name, "CANBus name cannot be null");
            this.bus = new com.ctre.phoenix6.CANBus(name);
        }
    }

    // Swerve-specific CANBus configuration
    public static class SwerveCANBus extends CANBus {
        public final int gyroId;
        public final int frontLeftDriveId;
        public final int frontLeftSteerId;
        public final int frontLeftEncoderId;
        public final int frontRightDriveId;
        public final int frontRightSteerId;
        public final int frontRightEncoderId;
        public final int backLeftDriveId;
        public final int backLeftSteerId;
        public final int backLeftEncoderId;
        public final int backRightDriveId;
        public final int backRightSteerId;
        public final int backRightEncoderId;

        public SwerveCANBus(String name, int gyroId,
                            int frontLeftDriveId, int frontLeftSteerId, int frontLeftEncoderId,
                            int frontRightDriveId, int frontRightSteerId, int frontRightEncoderId,
                            int backLeftDriveId, int backLeftSteerId, int backLeftEncoderId,
                            int backRightDriveId, int backRightSteerId, int backRightEncoderId) {
            super(name);
            this.gyroId = gyroId;
            this.frontLeftDriveId = frontLeftDriveId;
            this.frontLeftSteerId = frontLeftSteerId;
            this.frontLeftEncoderId = frontLeftEncoderId;
            this.frontRightDriveId = frontRightDriveId;
            this.frontRightSteerId = frontRightSteerId;
            this.frontRightEncoderId = frontRightEncoderId;
            this.backLeftDriveId = backLeftDriveId;
            this.backLeftSteerId = backLeftSteerId;
            this.backLeftEncoderId = backLeftEncoderId;
            this.backRightDriveId = backRightDriveId;
            this.backRightSteerId = backRightSteerId;
            this.backRightEncoderId = backRightEncoderId;
        }
    }

    // Subsystem factory interface
    private interface SubsystemFactory {
        Swerve  createSwerve();
        LEDMatrix createLEDMatrix();
        LEDStrip createLEDStrip();
        Vision createVision(VisionConsumer poseConsumer);
        ObjectVision createObjectVision();
        Shooter createShooter();
    }

    // OmegaBot factory
    private static class OmegaBotFactory implements SubsystemFactory {
        private final SwerveCANBus swerveCANBus;
        public OmegaBotFactory() {
            this.swerveCANBus = new SwerveCANBus(
                TunerConstants.DrivetrainConstants.CANBusName,
                TunerConstants.DrivetrainConstants.Pigeon2Id,
                TunerConstants.FrontLeft.DriveMotorId,
                TunerConstants.FrontLeft.SteerMotorId,
                TunerConstants.FrontLeft.EncoderId,
                TunerConstants.FrontRight.DriveMotorId,
                TunerConstants.FrontRight.SteerMotorId,
                TunerConstants.FrontRight.EncoderId,
                TunerConstants.BackLeft.DriveMotorId,
                TunerConstants.BackLeft.SteerMotorId,
                TunerConstants.BackLeft.EncoderId,
                TunerConstants.BackRight.DriveMotorId,
                TunerConstants.BackRight.SteerMotorId,
                TunerConstants.BackRight.EncoderId
            );
        }

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new GyroIOPigeon2(swerveCANBus.gyroId, swerveCANBus.bus),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return new LEDMatrix(8, 32);
        }

        @Override
        public LEDStrip createLEDStrip() {
            return new LEDStrip();
        }

        @Override
        public Vision createVision(VisionConsumer poseConsumer) {
            return new Vision(poseConsumer, false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return new ObjectVision();
        }

        @Override
        public Shooter createShooter() {
            return null;
        }
    }

    // AlphaBot factory
    private static class AlphaBotFactory implements SubsystemFactory {
        private final SwerveCANBus swerveCANBus;

        public AlphaBotFactory() {
            this.swerveCANBus = new SwerveCANBus(
                TunerConstantsAlpha.DrivetrainConstants.CANBusName,
                TunerConstantsAlpha.DrivetrainConstants.Pigeon2Id,
                TunerConstantsAlpha.FrontLeft.DriveMotorId,
                TunerConstantsAlpha.FrontLeft.SteerMotorId,
                TunerConstantsAlpha.FrontLeft.EncoderId,
                TunerConstantsAlpha.FrontRight.DriveMotorId,
                TunerConstantsAlpha.FrontRight.SteerMotorId,
                TunerConstantsAlpha.FrontRight.EncoderId,
                TunerConstantsAlpha.BackLeft.DriveMotorId,
                TunerConstantsAlpha.BackLeft.SteerMotorId,
                TunerConstantsAlpha.BackLeft.EncoderId,
                TunerConstantsAlpha.BackRight.DriveMotorId,
                TunerConstantsAlpha.BackRight.SteerMotorId,
                TunerConstantsAlpha.BackRight.EncoderId
            );
        }

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new GyroIOPigeon2(swerveCANBus.gyroId, swerveCANBus.bus),
                new ModuleIOTalonFX(TunerConstantsAlpha.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsAlpha.FrontRight),
                new ModuleIOTalonFX(TunerConstantsAlpha.BackLeft),
                new ModuleIOTalonFX(TunerConstantsAlpha.BackRight));
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return new LEDMatrix(8, 32);
        }

        @Override
        public LEDStrip createLEDStrip() {
            return new LEDStrip();
        }

        @Override
        public Vision createVision(VisionConsumer poseConsumer) {
            return new Vision(poseConsumer, false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null; // Not available on AlphaBot
        }

        @Override
        public Shooter createShooter() {
            return null;
        }
    }

    // Last year robot factory
    private static class LastYearFactory implements SubsystemFactory {
        private final SwerveCANBus swerveCANBus;

        public LastYearFactory() {
            this.swerveCANBus = new SwerveCANBus(
                TunerConstantsLastYear.DrivetrainConstants.CANBusName,
                TunerConstantsLastYear.DrivetrainConstants.Pigeon2Id,
                TunerConstantsLastYear.FrontLeft.DriveMotorId,
                TunerConstantsLastYear.FrontLeft.SteerMotorId,
                TunerConstantsLastYear.FrontLeft.EncoderId,
                TunerConstantsLastYear.FrontRight.DriveMotorId,
                TunerConstantsLastYear.FrontRight.SteerMotorId,
                TunerConstantsLastYear.FrontRight.EncoderId,
                TunerConstantsLastYear.BackLeft.DriveMotorId,
                TunerConstantsLastYear.BackLeft.SteerMotorId,
                TunerConstantsLastYear.BackLeft.EncoderId,
                TunerConstantsLastYear.BackRight.DriveMotorId,
                TunerConstantsLastYear.BackRight.SteerMotorId,
                TunerConstantsLastYear.BackRight.EncoderId
            );
        }

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new GyroIOPigeon2(swerveCANBus.gyroId, swerveCANBus.bus),
                new ModuleIOTalonFX(TunerConstantsLastYear.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsLastYear.FrontRight),
                new ModuleIOTalonFX(TunerConstantsLastYear.BackLeft),
                new ModuleIOTalonFX(TunerConstantsLastYear.BackRight));
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return null;
        }

        @Override
        public LEDStrip createLEDStrip() {
            return new LEDStrip();
        }

        @Override
        public Vision createVision(VisionConsumer poseConsumer) {
            return new Vision(poseConsumer, false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Shooter createShooter() {
            return null;
        }
    }

    // SimBot factory
    private static class SimBotFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new GyroIOSim(Objects.requireNonNull(
                    Swerve.getDriveSimulation(),
                    "Drive simulation not initialized").getGyroSimulation()),
                new ModuleIOSim(Swerve.getDriveSimulation().getModules()[0]),
                new ModuleIOSim(Swerve.getDriveSimulation().getModules()[1]),
                new ModuleIOSim(Swerve.getDriveSimulation().getModules()[2]),
                new ModuleIOSim(Swerve.getDriveSimulation().getModules()[3]));
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return new LEDMatrix(8, 32);
        }

        @Override
        public LEDStrip createLEDStrip() {
            return null;
        }

        @Override
        public Vision createVision(VisionConsumer poseConsumer) {
            return new Vision(poseConsumer, true);
        }

        @Override
        public ObjectVision createObjectVision() {
            return new ObjectVision();
        }

        @Override
        public Shooter createShooter() {
            return null;
        }
    }

    // Replay factory
    private static class ReplayFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return new LEDMatrix(8, 32);
        }

        @Override
        public LEDStrip createLEDStrip() {
            return null;
        }

        @Override
        public Vision createVision(VisionConsumer poseConsumer) {
            return new Vision(poseConsumer, false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Shooter createShooter() {
            return null;
        }
    }
}