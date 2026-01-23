package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import org.steelhawks.Constants.*;
import org.steelhawks.generated.*;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeIO;
import org.steelhawks.subsystems.superstructure.ShooterSuperstructure;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIO;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOTalonFX;
import org.steelhawks.subsystems.superstructure.pivot.Pivot;
import org.steelhawks.subsystems.superstructure.pivot.PivotIO;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.superstructure.turret.TurretIO;
import org.steelhawks.subsystems.superstructure.turret.TurretIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.Vision.VisionConsumer;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;

import java.util.Objects;
import java.util.Optional;

public class RobotConfig {
    // Feature flags
    public final boolean hasSwerve;
    public final boolean hasLEDMatrix;
    public final boolean hasLEDStrip;
    public final boolean hasVision;
    public final boolean hasObjectVision;
    public final boolean hasAutos;
    public final boolean hasFlywheel;
    public final boolean hasTurret;
    public final boolean hasPivot;
    public final boolean hasIntake;

    // Subsystem factory
    private final SubsystemFactory factory;

    private RobotConfig(Builder builder) {
        this.hasSwerve = builder.hasSwerve;
        this.hasLEDMatrix = builder.hasLEDMatrix;
        this.hasLEDStrip = builder.hasLEDStrip;
        this.hasVision = builder.hasVision;
        this.hasObjectVision = builder.hasObjectVision;
        this.hasAutos = builder.hasAutos;
        this.hasFlywheel = builder.hasFlywheel;
        this.hasTurret = builder.hasTurret;
        this.hasPivot = builder.hasPivot;
        this.hasIntake = builder.hasIntake;
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

    public Optional<ShooterSuperstructure> createShooterSuperStructure() {
        if (hasFlywheel || hasTurret || hasPivot) {
            return Optional.ofNullable(factory.createShooterSuperstructure());
        }
        return Optional.empty();
    }

    public Optional<Intake> createIntake() {
        if (!hasIntake) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createIntake());
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
                .withFlywheel(true)
                .withTurret(true)
                .withPivot(true)
                .withIntake(true)
                .withAutos(true)
                .withFactory(new OmegaBotFactory())
                .build();

            case ALPHABOT -> new Builder()
                .withLEDMatrix(true)
                .withLEDStrip(false)
                .withVision(true)
                .withObjectVision(false)
                .withFlywheel(true)
                .withTurret(true)
                .withPivot(false)
                .withIntake(true)
                .withAutos(true)
                .withFactory(new AlphaBotFactory())
                .build();

            case CHASSIS -> new Builder()
                .withLEDMatrix(false)
                .withLEDStrip(false)
                .withVision(true)
                .withObjectVision(true)
                .withFlywheel(false)
                .withTurret(false)
                .withPivot(false)
                .withAutos(false)
                .withFactory(new ChassisBotFactory())
                .build();

            case LAST_YEAR -> new Builder()
                .withLEDMatrix(false)
                .withLEDStrip(true)
                .withVision(true)
                .withObjectVision(false)
                .withFlywheel(false)
                .withTurret(false)
                .withPivot(false)
                .withIntake(false)
                .withAutos(false)
                .withFactory(new LastYearFactory())
                .build();

            case TEST_BOARD -> new Builder()
                .withSwerve(false)
                .withLEDMatrix(false)
                .withLEDStrip(false)
                .withVision(false)
                .withObjectVision(false)
                .withFlywheel(false)
                .withTurret(false)
                .withPivot(false)
                .withAutos(false)
                .withFactory(new TestBoardFactory())
                .build();

            case SIMBOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(true)
                .withFlywheel(true)
                .withTurret(true)
                .withPivot(true)
                .withIntake(true)
                .withAutos(true)
                .withFactory(new SimBotFactory())
                .build();
        };
    }

    private static RobotConfig getReplayConfig() {
        return switch (Constants.getRobot()) {
            case OMEGABOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(false)
                .withFlywheel(true)
                .withTurret(true)
                .withPivot(true)
                .withIntake(true)
                .withAutos(true)
                .withFactory(new ReplayFactory())
                .build();

            case ALPHABOT -> new Builder()
                .withLEDMatrix(false)
                .withVision(true)
                .withObjectVision(true)
                .withFlywheel(true)
                .withTurret(true)
                .withPivot(false)
                .withIntake(true)
                .withAutos(true)
                .withFactory(new ReplayFactory())
                .build();

            default -> new Builder()
                .withLEDMatrix(false)
                .withLEDStrip(true)
                .withVision(true)
                .withObjectVision(true)
                .withFlywheel(false)
                .withTurret(false)
                .withPivot(false)
                .withIntake(true)
                .withAutos(true)
                .withFactory(new ReplayFactory())
                .build();
        };
    }

    // Builder pattern
    public static class Builder {
        private boolean hasSwerve = true;
        private boolean hasLEDMatrix = false;
        private boolean hasLEDStrip = false;
        private boolean hasVision = false;
        private boolean hasObjectVision = false;
        private boolean hasFlywheel = false;
        private boolean hasTurret = false;
        private boolean hasPivot = false;
        private boolean hasIntake = false;
        private boolean hasAutos = false;
        private SubsystemFactory factory = null;

        public Builder withSwerve(boolean enabled) {
            this.hasSwerve = enabled;
            return this;
        }

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

        public Builder withFlywheel(boolean enabled) {
            this.hasFlywheel = enabled;
            return this;
        }

        public Builder withTurret(boolean enabled) {
            this.hasTurret = enabled;
            return this;
        }

        public Builder withPivot(boolean enabled) {
            this.hasPivot = enabled;
            return this;
        }

        public Builder withIntake(boolean enabled) {
            this.hasIntake = enabled;
            return this;
        }

        public Builder withAutos(boolean enabled) {
            this.hasAutos = enabled;
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
        Swerve createSwerve();
        LEDMatrix createLEDMatrix();
        LEDStrip createLEDStrip();
        Vision createVision(VisionConsumer poseConsumer);
        ObjectVision createObjectVision();
        ShooterSuperstructure createShooterSuperstructure();
        Intake createIntake();
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
        public ShooterSuperstructure createShooterSuperstructure() {
            return new ShooterSuperstructure(
                new Flywheel(new FlywheelIO() {}),
                new Turret(new TurretIO() {}),
                new Pivot(new PivotIO() {}));
        }

        @Override
        public Intake createIntake() {
            return new Intake(new IntakeIO() {});
        }
    }

    // AlphaBot factory
    private static class AlphaBotFactory implements SubsystemFactory {
        private final SwerveCANBus swerveCANBus;
        private final CANBus flywheelCANbus = new CANBus("");

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
        public ShooterSuperstructure createShooterSuperstructure() {
            return new ShooterSuperstructure(
                new Flywheel(new FlywheelIOTalonFX(flywheelCANbus)),
                new Turret(new TurretIO() {}),
                null);
        }

        @Override
        public Intake createIntake() {
            return new Intake(new IntakeIO() {});
        }
    }

    private static class ChassisBotFactory implements SubsystemFactory {
        private final SwerveCANBus swerveCANBus;

        public ChassisBotFactory() {
            this.swerveCANBus = new SwerveCANBus(
                TunerConstantsChassis.DrivetrainConstants.CANBusName,
                TunerConstantsChassis.DrivetrainConstants.Pigeon2Id,
                TunerConstantsChassis.FrontLeft.DriveMotorId,
                TunerConstantsChassis.FrontLeft.SteerMotorId,
                TunerConstantsChassis.FrontLeft.EncoderId,
                TunerConstantsChassis.FrontRight.DriveMotorId,
                TunerConstantsChassis.FrontRight.SteerMotorId,
                TunerConstantsChassis.FrontRight.EncoderId,
                TunerConstantsChassis.BackLeft.DriveMotorId,
                TunerConstantsChassis.BackLeft.SteerMotorId,
                TunerConstantsChassis.BackLeft.EncoderId,
                TunerConstantsChassis.BackRight.DriveMotorId,
                TunerConstantsChassis.BackRight.SteerMotorId,
                TunerConstantsChassis.BackRight.EncoderId
            );
        }

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new GyroIOPigeon2(swerveCANBus.gyroId, swerveCANBus.bus),
                new ModuleIOTalonFX(TunerConstantsChassis.FrontLeft),
                new ModuleIOTalonFX(TunerConstantsChassis.FrontRight),
                new ModuleIOTalonFX(TunerConstantsChassis.BackLeft),
                new ModuleIOTalonFX(TunerConstantsChassis.BackRight));
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return null;
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
            return new ObjectVision();
        }

        @Override
        public ShooterSuperstructure createShooterSuperstructure() {
            return null;
        }

        @Override
        public Intake createIntake() {
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
        public ShooterSuperstructure createShooterSuperstructure() {
            return null;
        }

        @Override
        public Intake createIntake() {
            return null;
        }
    }

    private static class TestBoardFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return null;
        }

        @Override
        public LEDMatrix createLEDMatrix() {
            return null;
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
        public ShooterSuperstructure createShooterSuperstructure() {
            return new ShooterSuperstructure(
                new Flywheel(new FlywheelIO() {}),
                new Turret(new TurretIOTalonFX(new CANBus(""))),
                new Pivot(new PivotIO() {}));
        }

        @Override
        public Intake createIntake() {
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
        public ShooterSuperstructure createShooterSuperstructure() {
            return null;
        }

        @Override
        public Intake createIntake() {
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
        public ShooterSuperstructure createShooterSuperstructure() {
            return new ShooterSuperstructure(
                new Flywheel(new FlywheelIO() {}),
                new Turret(new TurretIO() {}),
                new Pivot(new PivotIO() {}));
        }

        @Override
        public Intake createIntake() {
            return new Intake(new IntakeIO() {});
        }
    }
}
