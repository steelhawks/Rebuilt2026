    package org.steelhawks;

    import org.steelhawks.Constants.*;
    import org.steelhawks.generated.*;
    import org.steelhawks.subsystems.Superstructure.flywheel.Flywheel;
    import org.steelhawks.subsystems.Superstructure.flywheel.FlywheelIO;
    import org.steelhawks.subsystems.Superstructure.flywheel.FlywheelIOSim;
    import org.steelhawks.subsystems.intake.*;
    import org.steelhawks.subsystems.led.LEDMatrix;
    import org.steelhawks.subsystems.led.LEDStrip;
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
        public final boolean hasShooter;// hasFlywheel on main
        public final boolean hasIntake;
        public final boolean hasFlywheel;

        private final BuilderConstants.IntakeConstants intakeConstants;
        private final BuilderConstants.FlywheelConstants flywheelConstants;

        // Subsystem factory
        private final SubsystemFactory factory;

        private RobotConfig(Builder builder) {
            this.hasLEDMatrix = builder.hasLEDMatrix;
            this.hasLEDStrip = builder.hasLEDStrip;
            this.hasVision = builder.hasVision;
            this.hasObjectVision = builder.hasObjectVision;
            this.hasAutos = builder.hasAutos;
            this.hasShooter = builder.hasShooter;// builder.hasFlywheel on main
            this.hasIntake = builder.hasIntake;
            this.intakeConstants = builder.intakeConstants;
            this.hasFlywheel = builder.hasFlywheel;
            this.flywheelConstants = builder.flywheelConstants;
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

        public Optional<Intake> createIntake() {
            if (!hasIntake) {
                return Optional.empty();
            }
            return Optional.ofNullable(factory.createIntake(intakeConstants));
        }

        public Optional<Flywheel> createFlywheel() {
            if (!hasFlywheel) {
                return Optional.empty();
            }
            return Optional.ofNullable(factory.createFlywheel(flywheelConstants));
        }

        public static RobotConfig getConfig() {
            if (Constants.getMode() == Mode.REPLAY) {
                return getReplayConfig();
            }

            return switch (Constants.getRobot()) {
                case OMEGABOT -> new Builder()
                        .withFlywheel(true, BuilderConstants.OmegaBot.FLYWHEEL)
                        .withShooter(true)
                    .withLEDMatrix(true)
                    .withVision(true)
                    .withObjectVision(true)
                    .withAutos(true)
                    .withFactory(new OmegaBotFactory())
                        .withIntake(true, BuilderConstants.OmegaBot.INTAKE)
                    .build();

                case ALPHABOT -> new Builder()
                        .withFlywheel(true, BuilderConstants.OmegaBot.FLYWHEEL)
                    .withLEDMatrix(true)
                    .withVision(true)
                    .withObjectVision(false)
                    .withAutos(true)
                        .withIntake(true, BuilderConstants.OmegaBot.INTAKE)
                    .withFactory(new AlphaBotFactory()).withShooter(true)
                    .build();

                case LAST_YEAR -> new Builder()
                        .withFlywheel(true, BuilderConstants.OmegaBot.FLYWHEEL)
                    .withLEDMatrix(true)
                    .withVision(true)
                    .withObjectVision(false)
                        .withIntake(true, BuilderConstants.OmegaBot.INTAKE)

                    .withAutos(true)
                    .withFactory(new LastYearFactory()).withShooter(false)
                    .build();

                case SIMBOT -> new Builder()
                        .withFlywheel(true, BuilderConstants.OmegaBot.FLYWHEEL)
                    .withLEDMatrix(true)
                    .withVision(true)
                    .withObjectVision(true)
                        .withIntake(true, BuilderConstants.OmegaBot.INTAKE)
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
                        .withIntake(true, BuilderConstants.OmegaBot.INTAKE)
                        .withFlywheel(true, BuilderConstants.OmegaBot.FLYWHEEL)
                    .withFactory(new ReplayFactory()).withShooter(true)
                    .build();

                default -> new Builder()
                    .withLEDMatrix(true)
                    .withVision(true)
                        .withIntake(true, BuilderConstants.OmegaBot.INTAKE)
                    .withObjectVision(false)
                    .withAutos(true)
                        .withFlywheel(true, BuilderConstants.OmegaBot.FLYWHEEL)
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
            private boolean hasIntake = false;
            private boolean hasFlywheel = false;
            private SubsystemFactory factory = null;

            private BuilderConstants.IntakeConstants intakeConstants;
            private BuilderConstants.FlywheelConstants flywheelConstants;

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

            public Builder withIntake(boolean enabled, BuilderConstants.IntakeConstants intakeConstants) {
                this.hasIntake = enabled;
                this.intakeConstants = intakeConstants;
                return this;

            }

            public Builder withFlywheel(boolean enabled, BuilderConstants.FlywheelConstants flywheelConstants) {
                this.hasFlywheel = enabled;
                this.flywheelConstants = flywheelConstants;
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

        public static class TurretCANBus extends CANBus {

            public final int turretMotorID;
            public final int turretEncoderID;
            public final int flywheelID;
            public final int hoodMotorID;
            public final int hoodEncoderID;


            public TurretCANBus(String name, int turretMotorID, int turretEncoderID, int flywheelID, int hoodEncoderID, int hoodMotorID) {
                super(name);
                this.turretMotorID = turretMotorID;
                this.turretEncoderID = turretEncoderID;
                this.flywheelID = flywheelID;
                this.hoodEncoderID = hoodEncoderID;
                this.hoodMotorID = hoodMotorID;
            }
        }

        public static class IntakeCANBus extends CANBus {

            public final int leftIntakeMotorID;
            public final int leftIntakeEncoderID;

            public final int rightIntakeMotorID;
            public final int rightIntakeEncoderID;

            public final int rollerMotorID;
            public final int rollerEncoderID;

            public IntakeCANBus(String name, int leftIntakeMotorID, int leftIntakeEncoderID, int rightIntakeMotorID, int rightIntakeEncoderID, int rollerMotorID, int rollerEncoderID) {
                super(name);
                this.leftIntakeMotorID = leftIntakeMotorID;
                this.leftIntakeEncoderID = -leftIntakeEncoderID;
                this.rightIntakeMotorID = rightIntakeMotorID;
                this.rightIntakeEncoderID = -rightIntakeEncoderID;
                this.rollerMotorID = rollerMotorID;
                this.rollerEncoderID = -rollerEncoderID;
            }
        }





        // Subsystem factory interface
        private interface SubsystemFactory {
            Swerve  createSwerve();
            LEDMatrix createLEDMatrix();
            LEDStrip createLEDStrip();
            Vision createVision(VisionConsumer poseConsumer);
            ObjectVision createObjectVision();
            Intake createIntake(BuilderConstants.IntakeConstants c);
            Flywheel createFlywheel(BuilderConstants.FlywheelConstants f);
            /*
             SuperStructure createSuperStructure();
             Intake createIntake();
            */
        }

        // OmegaBot factory
        private static class OmegaBotFactory implements SubsystemFactory {

            private final SwerveCANBus swerveCANBus;
    //        private final TurretCANBus turretCANBus;
            private final IntakeCANBus intakeCANBus;

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

    /*
                this.turretCANBus = new TurretCANBus(
                        CANBusList.kTurretBus,
                        SuperStructureConstants.TurretMotorID,
                        SuperStructureConstants.TurretEncoderID,
                        SuperStructureConstants.FlywheelID,
                        SuperStructureConstants.HoodMotorID,
                        SuperStructureConstants.HoodEncoderID,

                );
    */
                this.intakeCANBus = new IntakeCANBus(
                        IntakeConstants.EXTENSION_CANBUS_NAME,

                        IntakeConstants.EXTENSION_LEFT_MOTOR_ID,
                        IntakeConstants.EXTENSION_LEFT_ENCODER_ID,


                        IntakeConstants.EXTENSION_RIGHT_MOTOR_ID,
                        IntakeConstants.EXTENSION_RIGHT_ENCODER_ID,

                        IntakeConstants.ROLLER_MOTOR_ID,
                        IntakeConstants.ROLLER_ENCODER_ID

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
            public Intake createIntake(BuilderConstants.IntakeConstants c) {
                return null;
            }

            @Override
            public Flywheel createFlywheel(BuilderConstants.FlywheelConstants f) {
                return null;
            }


        }

        // AlphaBot factory
        private static class AlphaBotFactory implements SubsystemFactory {
            private final SwerveCANBus swerveCANBus;
            private final IntakeCANBus intakeCANBus;

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
     /**
                this.turretCANBus = new TurretCANBus(
                        CANBusList.kTurretBus,
                        SuperStructureConstants.TurretMotorID,
                        SuperStructureConstants.TurretEncoderID,
                        SuperStructureConstants.FlywheelID,
                        SuperStructureConstants.HoodMotorID,
                        SuperStructureConstants.HoodEncoderID,

                        );
    */
                this.intakeCANBus = new IntakeCANBus(
                        IntakeConstants.EXTENSION_CANBUS_NAME,

                        IntakeConstants.EXTENSION_LEFT_MOTOR_ID,
                        IntakeConstants.EXTENSION_LEFT_ENCODER_ID,


                        IntakeConstants.EXTENSION_RIGHT_MOTOR_ID,
                        IntakeConstants.EXTENSION_RIGHT_ENCODER_ID,

                        IntakeConstants.ROLLER_MOTOR_ID,
                        IntakeConstants.ROLLER_ENCODER_ID

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
            public Intake createIntake(BuilderConstants.IntakeConstants c) {
                return null;
            }

            @Override
            public Flywheel createFlywheel(BuilderConstants.FlywheelConstants f) {
                return null;
            }


        }

        // Last year robot factory
        private static class LastYearFactory implements SubsystemFactory {
            private final SwerveCANBus swerveCANBus;
            private final IntakeCANBus intakeCANBus;

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

                /**
                 this.turretCANBus = new TurretCANBus(
                 CANBusList.kTurretBus,
                 SuperStructureConstants.TurretMotorID,
                 SuperStructureConstants.TurretEncoderID,
                 SuperStructureConstants.FlywheelID,
                 SuperStructureConstants.HoodMotorID,
                 SuperStructureConstants.HoodEncoderID,

                 );
                 */
                this.intakeCANBus = new IntakeCANBus(
                        IntakeConstants.EXTENSION_CANBUS_NAME,

                        IntakeConstants.EXTENSION_LEFT_MOTOR_ID,
                        IntakeConstants.EXTENSION_LEFT_ENCODER_ID,


                        IntakeConstants.EXTENSION_RIGHT_MOTOR_ID,
                        IntakeConstants.EXTENSION_RIGHT_ENCODER_ID,

                        IntakeConstants.ROLLER_MOTOR_ID,
                        IntakeConstants.ROLLER_ENCODER_ID

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
            public Intake createIntake(BuilderConstants.IntakeConstants c) {
                return null;
            }

            @Override
            public Flywheel createFlywheel(BuilderConstants.FlywheelConstants f) {
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
                return new LEDMatrix(32, 8);
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
            public Intake createIntake(BuilderConstants.IntakeConstants c) {
                return new Intake(new IntakeIOSim(c));
            }

            @Override
            public Flywheel createFlywheel(BuilderConstants.FlywheelConstants f) {
                return new Flywheel(new FlywheelIOSim(f), f);
            }

        }

        public static class CANBusList {
            public static final com.ctre.phoenix6.CANBus kDrivetrainBus = new com.ctre.phoenix6.CANBus("NoImJacobivore");
            public static final com.ctre.phoenix6.CANBus kTurretBus = new com.ctre.phoenix6.CANBus("Farhanivore");
            public static final com.ctre.phoenix6.CANBus kIntakeBus = new com.ctre.phoenix6.CANBus("Parmesanivore");
            public static final com.ctre.phoenix6.CANBus kRioBus = new com.ctre.phoenix6.CANBus("rio");
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
            public Intake createIntake(BuilderConstants.IntakeConstants c) {
                return null;
            }

            @Override
            public Flywheel createFlywheel(BuilderConstants.FlywheelConstants f) {
                return null;
            }


        }
    }