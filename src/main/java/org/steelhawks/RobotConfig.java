package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.Constants.*;
import org.steelhawks.generated.*;
import org.steelhawks.subsystems.indexer.IndexerIO;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeIO;
import org.steelhawks.subsystems.intake.IntakeIOSim;
import org.steelhawks.subsystems.intake.IntakeIOTalonFX;
import org.steelhawks.subsystems.oldintake.*;
import org.steelhawks.subsystems.oldintake.OldIntakeIOTalonFX;
import org.steelhawks.subsystems.oldintake.OldIntakeIO;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.indexer.IndexerIOSim;
import org.steelhawks.subsystems.indexer.IndexerIOTalonFX;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.led.LEDStrip;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIO;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOSim;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOTalonFX;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.hood.HoodIO;
import org.steelhawks.subsystems.superstructure.hood.HoodIOSim;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.superstructure.turret.TurretIO;
import org.steelhawks.subsystems.superstructure.turret.TurretIOSim;
import org.steelhawks.subsystems.superstructure.turret.TurretIOTalonFX;
import org.steelhawks.subsystems.swerve.*;
import org.steelhawks.subsystems.vision.*;
import org.steelhawks.subsystems.vision.objdetect.ObjectVision;

import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

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
    public final boolean hasHood;
    public final boolean hasOldIntake;
    public final boolean hasIntake;
    public final boolean hasIndexer;

    // Subsystem factory
    private final SubsystemFactory factory;

    public record System(SubsystemBase subsystem, CANBus canBus) {}

    private RobotConfig(Builder builder) {
        this.hasSwerve = builder.hasSwerve;
        this.hasLEDMatrix = builder.hasLEDMatrix;
        this.hasLEDStrip = builder.hasLEDStrip;
        this.hasVision = builder.hasVision;
        this.hasObjectVision = builder.hasObjectVision;
        this.hasAutos = builder.hasAutos;
        this.hasFlywheel = builder.hasFlywheel;
        this.hasTurret = builder.hasTurret;
        this.hasHood = builder.hasHood;
        this.hasOldIntake = builder.hasOldIntake;
        this.hasIntake = builder.hasIntake;
        this.hasIndexer = builder.hasIndexer;
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

    public Optional<Vision> createVision() {
        if (!hasVision) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createVision());
    }

    public Optional<ObjectVision> createObjectVision() {
        if (!hasObjectVision) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createObjectVision());
    }

    public Optional<Flywheel> createFlywheel() {
        if (!hasTurret) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createFlywheel());
    }

    public Optional<Turret> createTurret(Supplier<Pose2d> poseSupplier) {
        if (!hasTurret) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createTurret(poseSupplier));
    }

    public Optional<Hood> createHood() {
        if (!hasTurret) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createHood());
    }

    public Optional<OldIntake> createOldIntake() {
        if (!hasOldIntake) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createOldIntake());
    }

    public Optional<Indexer> createIndexer() {
        if (!hasIndexer) {
            return Optional.empty();
        }
        return Optional.ofNullable(factory.createIndexer());
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
                .withVision(false)
                .withObjectVision(false)
                .withFlywheel(false)
                .withTurret(false)
                .withHood(false)
                .withOldIntake(false)
                .withIntake(false)
                .withIndexer(false)
                .withAutos(false)
                .withFactory(new OmegaBotFactory())
                .build();

            case ALPHABOT -> new Builder()
                .withLEDMatrix(false)
                .withLEDStrip(false)
                .withVision(true)
                .withObjectVision(false)
                .withFlywheel(true)
                .withTurret(true)
                .withHood(false)
                .withOldIntake(false)
                .withIntake(false)
                .withIndexer(true)
                .withAutos(false)
                .withFactory(new AlphaBotFactory())
                .build();

            case CHASSIS -> new Builder()
                .withLEDMatrix(false)
                .withLEDStrip(false)
                .withVision(true)
                .withObjectVision(false)
                .withFlywheel(false)
                .withTurret(false)
                .withHood(false)
                .withOldIntake(false)
                .withIntake(false)
                .withIndexer(false)
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
                .withHood(false)
                .withOldIntake(false)
                .withIntake(false)
                .withIndexer(false)
                .withAutos(false)
                .withFactory(new LastYearFactory())
                .build();

            case TEST_BOARD -> new Builder()
                .withSwerve(false)
                .withLEDMatrix(false)
                .withLEDStrip(false)
                .withVision(false)
                .withObjectVision(false)
                .withFlywheel(true)
                .withTurret(true)
                .withHood(false)
                .withOldIntake(false)
                .withIntake(false)
                .withIndexer(false)
                .withAutos(false)
                .withFactory(new TestBoardFactory())
                .build();

            case SIMBOT -> new Builder()
                .withLEDMatrix(true)
                .withVision(true)
                .withObjectVision(true)
                .withFlywheel(true)
                .withTurret(true)
                .withHood(true)
                .withOldIntake(false)
                .withIntake(true)
                .withIndexer(true)
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
                .withHood(true)
                .withOldIntake(false)
                .withIntake(true)
                .withIndexer(true)
                .withAutos(true)
                .withFactory(new ReplayFactory())
                .build();

            case ALPHABOT -> new Builder()
                .withLEDMatrix(false)
                .withVision(true)
                .withObjectVision(true)
                .withFlywheel(true)
                .withTurret(true)
                .withHood(false)
                .withOldIntake(false)
                .withIntake(true)
                .withIndexer(true)
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
                .withHood(false)
                .withOldIntake(false)
                .withIntake(true)
                .withIndexer(true)
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
        private boolean hasHood = false;
        private boolean hasOldIntake = false;
        private boolean hasIntake = false;
        private boolean hasIndexer = false;
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

        public Builder withHood(boolean enabled) {
            this.hasHood = enabled;
            return this;
        }

        public Builder withOldIntake(boolean enabled) {
            this.hasOldIntake = enabled;
            return this;
        }

        public Builder withIntake(boolean enabled) {
            this.hasIntake = enabled;
            return this;
        }

        public Builder withIndexer(boolean enabled) {
            this.hasIndexer = enabled;
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

    public static class CANBusList {
        public static final com.ctre.phoenix6.CANBus kDrivetrainBus = new com.ctre.phoenix6.CANBus("NoImJacobivore");
        public static final com.ctre.phoenix6.CANBus kTurretBus = new com.ctre.phoenix6.CANBus("Farhanivore");
        public static final com.ctre.phoenix6.CANBus kRioBus = new com.ctre.phoenix6.CANBus("");
    }

    // Subsystem factory interface
    private interface SubsystemFactory {
        Swerve createSwerve();
        LEDMatrix createLEDMatrix();
        LEDStrip createLEDStrip();
        Vision createVision();
        ObjectVision createObjectVision();
        Flywheel createFlywheel();
        Turret createTurret(Supplier<Pose2d> poseSupplier);
        Hood createHood();
        OldIntake createOldIntake();
        Intake createIntake();
        Indexer createIndexer();
    }

    // OmegaBot factory
    private static class OmegaBotFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return new Swerve(
                CANBusList.kDrivetrainBus,
                new GyroIOPigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, CANBusList.kDrivetrainBus),
                new ModuleIOTalonFX(TunerConstants.FrontLeft, CANBusList.kDrivetrainBus),
                new ModuleIOTalonFX(TunerConstants.FrontRight, CANBusList.kDrivetrainBus),
                new ModuleIOTalonFX(TunerConstants.BackLeft, CANBusList.kDrivetrainBus),
                new ModuleIOTalonFX(TunerConstants.BackRight, CANBusList.kDrivetrainBus));
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
        public Vision createVision() {
            return null;
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Flywheel createFlywheel() {
            return null;
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return null;
        }

        @Override
        public Hood createHood() {
            return null;
        }

        @Override
        public OldIntake createOldIntake() {
            return null;
        }

        @Override
        public Intake createIntake() {
            return null;
        }

        @Override
        public Indexer createIndexer() {
            return null;
        }
    }

    // AlphaBot factory
    private static class AlphaBotFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return new Swerve(
                CANBusList.kRioBus,
                new GyroIOPigeon2(TunerConstantsAlpha.DrivetrainConstants.Pigeon2Id, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsAlpha.FrontLeft, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsAlpha.FrontRight, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsAlpha.BackLeft, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsAlpha.BackRight, CANBusList.kRioBus));
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
        public Vision createVision() {
            return new Vision(false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Flywheel createFlywheel() {
            return new Flywheel(new FlywheelIOTalonFX(CANBusList.kRioBus));
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return new Turret(new TurretIOTalonFX(CANBusList.kRioBus), poseSupplier);
        }

        @Override
        public Hood createHood() {
            return null;
        }

        @Override
        public OldIntake createOldIntake() {
//            return new OldIntake(new OldIntakeIOTalonFX(CANBusList.kRioBus));
            return null;
        }

        @Override
        public Intake createIntake() {
//            return new Intake(new IntakeIOTalonFX(CANBusList.kRioBus));
            return null;
        }

        @Override
        public Indexer createIndexer() { return new Indexer(new IndexerIOTalonFX(CANBusList.kRioBus)); }
    }

    private static class ChassisBotFactory implements SubsystemFactory {

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                CANBusList.kRioBus,
                new GyroIOPigeon2(TunerConstantsChassis.DrivetrainConstants.Pigeon2Id, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsChassis.FrontLeft, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsChassis.FrontRight, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsChassis.BackLeft, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsChassis.BackRight, CANBusList.kRioBus));
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
        public Vision createVision() {
            return new Vision(false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return new ObjectVision();
        }

        @Override
        public Flywheel createFlywheel() {
            return null;
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return null;
        }

        @Override
        public Hood createHood() {
            return null;
        }

        @Override
        public OldIntake createOldIntake() {
            return null;
        }

        @Override
        public Intake createIntake() {
            return null;
        }

        @Override
        public Indexer createIndexer() { return null; }
    }

    private static class LastYearFactory implements SubsystemFactory {

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                CANBusList.kRioBus,
                new GyroIOPigeon2(TunerConstantsLastYear.DrivetrainConstants.Pigeon2Id, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsLastYear.FrontLeft, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsLastYear.FrontRight, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsLastYear.BackLeft, CANBusList.kRioBus),
                new ModuleIOTalonFX(TunerConstantsLastYear.BackRight, CANBusList.kRioBus));
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
        public Vision createVision() {
            return new Vision(false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Flywheel createFlywheel() {
            return null;
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return null;
        }

        @Override
        public Hood createHood() {
            return null;
        }

        @Override
        public OldIntake createOldIntake() {
            return null;
        }

        @Override
        public Intake createIntake() {
            return null;
        }

        @Override
        public Indexer createIndexer() { return null; }
    }

    private static class TestBoardFactory implements SubsystemFactory {

        @Override
        public Swerve createSwerve() {
            return new Swerve(
                CANBusList.kRioBus,
                new GyroIOSim(Objects.requireNonNull(
                Swerve.getDriveSimulation(), "Drive simulation not initialized")
                    .getGyroSimulation()),
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
        public Vision createVision() {
            return new Vision(false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Flywheel createFlywheel() {
            return new Flywheel(new FlywheelIOTalonFX(CANBusList.kRioBus));
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return new Turret(new TurretIOTalonFX(CANBusList.kRioBus), poseSupplier);
        }

        @Override
        public Hood createHood() {
            return null;
        }

        @Override
        public OldIntake createOldIntake() {
            return null;
        }

        @Override
        public Intake createIntake() {
            return null;
        }

        @Override
        public Indexer createIndexer() { return null; }
    }

    private static class SimBotFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new com.ctre.phoenix6.CANBus(),
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
        public Vision createVision() {
            return new Vision(true);
        }

        @Override
        public ObjectVision createObjectVision() {
            return new ObjectVision();
        }

        @Override
        public Flywheel createFlywheel() {
            return new Flywheel(new FlywheelIOSim());
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return new Turret(new TurretIOSim(), poseSupplier);
        }

        @Override
        public Hood createHood() {
            return new Hood(new HoodIOSim());
        }

        @Override
        public OldIntake createOldIntake() {
            return null;
        }

        @Override
        public Intake createIntake() {
            return new Intake(new IntakeIOSim());
        }

        @Override
        public Indexer createIndexer() { return new Indexer(new IndexerIOSim()); }
    }

    private static class ReplayFactory implements SubsystemFactory {
        @Override
        public Swerve createSwerve() {
            return new Swerve(
                new com.ctre.phoenix6.CANBus(),
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
        public Vision createVision() {
            return new Vision(false);
        }

        @Override
        public ObjectVision createObjectVision() {
            return null;
        }

        @Override
        public Flywheel createFlywheel() {
            return new Flywheel(new FlywheelIO() {});
        }

        @Override
        public Turret createTurret(Supplier<Pose2d> poseSupplier) {
            return new Turret(new TurretIO() {}, poseSupplier);
        }

        @Override
        public Hood createHood() {
            return new Hood(new HoodIO() {});
        }

        @Override
        public OldIntake createOldIntake() {
            return new OldIntake(new OldIntakeIO() {});
        }

        @Override
        public Intake createIntake() {
            return new Intake(new IntakeIO() {});
        }

        @Override
        public Indexer createIndexer() { return new Indexer(new IndexerIO() {}); }
    }
}
