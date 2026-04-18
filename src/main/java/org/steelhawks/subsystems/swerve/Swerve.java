package org.steelhawks.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.steelhawks.*;
import org.steelhawks.Constants.*;

import java.util.Objects;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsChassis;
import org.steelhawks.generated.TunerConstantsLastYear;
import org.steelhawks.util.LocalADStarAK;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.LoopTimeUtil;
import org.steelhawks.util.SwerveDriveController;

public class Swerve extends SubsystemBase {

    private static final double SLOW_SPEED_MULTIPLIER = 0.3;
    private static final double SPEED_MULTIPLIER = 1.0;
    private static final SwerveModuleState[] EMPTY_MODULE_STATES = new SwerveModuleState[0];
    private boolean isPathfinding = false;
    private boolean requestSlowMode = false;

    public static final double ODOMETRY_FREQUENCY;
    public static final double DRIVE_BASE_RADIUS;

    // PathPlanner config constants
    private static final DCMotor DRIVE_MOTOR;
    private static final DCMotor TURN_MOTOR;
    private static final double ROBOT_MASS_KG;
    private static final double ROBOT_MOI;
    private static final double WHEEL_COF;
    private static final RobotConfig PP_CONFIG;

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG;
    private static final SwerveDriveSimulation DRIVE_SIMULATION;

    private ChassisSpeeds previousSetpoint = new ChassisSpeeds();
    private ChassisSpeeds currentSetpoint = new ChassisSpeeds();
    private double previousSetpointTime = 0.0;

    // Collision constants
    private final LoggedTunableNumber COLLISION_ACCEL_THRESHOLD =
        new LoggedTunableNumber("Swerve/CollisionAccelThreshold", 0.3);
    private double previousAx = 0.0;
    private double previousAy = 0.0;

    private final LoggedTunableNumber COLLISION_ANG_ACCEL_THRESHOLD =
        new LoggedTunableNumber("Swerve/CollisionAngAccelThreshold", 0.0);
    private double previousAngularVelocityZ = 0.0;
    private double previousAngularAccelZ = 0.0;

    private final LoggedTunableNumber COLLISION_JERK_THRESHOLD =
        new LoggedTunableNumber("Swerve/CollisionJerkThreshold", 50.0);
    private final LoggedTunableNumber COLLISION_ANG_JERK_THRESHOLD =
        new LoggedTunableNumber("Swerve/CollisionAngJerkThreshold", 100.0);
    private final LoggedTunableNumber COMMANDED_ACCEL_FILTER_THRESHOLD =
        new LoggedTunableNumber("Swerve/CommandedAccelFilterThreshold", 4.0);

    private final LoggedTunableNumber BUMP_ANGLE_THRESHOLD =
        new LoggedTunableNumber("Swerve/BumpAngleThreshold", 12.0);

    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR

    private final SwerveModuleState[] moduleStatesBuffer = {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    private final SwerveModulePosition[] modulePositionsBuffer = {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private final SwerveModulePosition[] odometryPositionsBuffer = {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private final SwerveModulePosition[] odometryDeltasBuffer = {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    private final SysIdRoutine driveSysId;
    private final SysIdRoutine turnSysId;
    private final SysIdRoutine angularSysId;
//    private final Alert gyroDisconnectedAlert =
//        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveController autonController =
        new SwerveDriveController(
            new PIDController(AutonConstants.TRANSLATION_KP.getAsDouble(), AutonConstants.TRANSLATION_KI.getAsDouble(), AutonConstants.TRANSLATION_KD.getAsDouble()),
            new PIDController(AutonConstants.TRANSLATION_KP.getAsDouble(), AutonConstants.TRANSLATION_KI.getAsDouble(), AutonConstants.TRANSLATION_KD.getAsDouble()),
            new PIDController(AutonConstants.ROTATION_KP.getAsDouble(), AutonConstants.ROTATION_KI.getAsDouble(), AutonConstants.ROTATION_KD.getAsDouble()))
            .withLinearTolerance(0.05)
            .withRotationalTolerance(Math.PI / 60.0);
    private final ProfiledPIDController mAlignController;
    private final Debouncer mAlignDebouncer;
    private final Debouncer collisionDebouncer;
    private final Debouncer bumpRisingDebouncer, bumpFallingDebouncer;

    private final SwerveDriveKinematics kinematics;
    private final SwerveModulePosition[] lastModulePositions;
    private Rotation2d rawGyroRotation = new Rotation2d();

    private final RobotState robotState = RobotState.getInstance();

    static {
        ODOMETRY_FREQUENCY = Constants.getRobot().equals(RobotType.OMEGABOT) ? 250.0 : 100.0;
        switch (Constants.getRobot()) {
            case ALPHABOT -> {
                DRIVE_BASE_RADIUS =
                    Math.max(
                        Math.max(
                            Math.hypot(TunerConstantsAlpha.FrontLeft.LocationX, TunerConstantsAlpha.FrontLeft.LocationY),
                            Math.hypot(TunerConstantsAlpha.FrontRight.LocationX, TunerConstantsAlpha.FrontRight.LocationY)),
                        Math.max(
                            Math.hypot(TunerConstantsAlpha.BackLeft.LocationX, TunerConstantsAlpha.BackLeft.LocationY),
                            Math.hypot(TunerConstantsAlpha.BackRight.LocationX, TunerConstantsAlpha.BackRight.LocationY)));
                DRIVE_MOTOR = DCMotor.getKrakenX60(1);
                TURN_MOTOR = DCMotor.getKrakenX60(1);
                ROBOT_MASS_KG = Units.lbsToKilograms(124.8);
                ROBOT_MOI = (1.0 / 12.0) * ROBOT_MASS_KG * (2 * Math.pow(Units.inchesToMeters(25), 2));
                WHEEL_COF = COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstantsAlpha.FrontLeft.WheelRadius,
                            TunerConstantsAlpha.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DRIVE_MOTOR
                                .withReduction(TunerConstantsAlpha.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsAlpha.FrontLeft.SlipCurrent,
                            1),
                        Objects.requireNonNull(getModuleTranslations()));
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DRIVE_MOTOR,
                                TURN_MOTOR,
                                TunerConstantsAlpha.FrontLeft.DriveMotorGearRatio,
                                TunerConstantsAlpha.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstantsAlpha.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstantsAlpha.FrontLeft.SteerFrictionVoltage),
                                Meters.of(TunerConstantsAlpha.FrontLeft.WheelRadius),
                                KilogramSquareMeters.of(TunerConstantsAlpha.FrontLeft.SteerInertia),
                                WHEEL_COF));
            }
            case LAST_YEAR -> {
                DRIVE_BASE_RADIUS =
                    Math.max(
                        Math.max(
                            Math.hypot(TunerConstantsLastYear.FrontLeft.LocationX, TunerConstantsLastYear.FrontLeft.LocationY),
                            Math.hypot(TunerConstantsLastYear.FrontRight.LocationX, TunerConstantsLastYear.FrontRight.LocationY)),
                        Math.max(
                            Math.hypot(TunerConstantsLastYear.BackLeft.LocationX, TunerConstantsLastYear.BackLeft.LocationY),
                            Math.hypot(TunerConstantsLastYear.BackRight.LocationX, TunerConstantsLastYear.BackRight.LocationY)));
                DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
                TURN_MOTOR = DCMotor.getKrakenX60Foc(1);
                ROBOT_MASS_KG = Units.lbsToKilograms(138 + (6.0 / 16.0));
                ROBOT_MOI = (1.0 / 12.0) * ROBOT_MASS_KG * (2 * Math.pow(Units.inchesToMeters(30), 2));
                WHEEL_COF = COTS.WHEELS.COLSONS.cof;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstantsLastYear.FrontLeft.WheelRadius,
                            TunerConstantsLastYear.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DRIVE_MOTOR
                                .withReduction(TunerConstantsLastYear.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsLastYear.FrontLeft.SlipCurrent,
                            1),
                        Objects.requireNonNull(getModuleTranslations()));
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DRIVE_MOTOR,
                                TURN_MOTOR,
                                TunerConstantsLastYear.FrontLeft.DriveMotorGearRatio,
                                TunerConstantsLastYear.FrontLeft.SteerMotorGearRatio,
                                Volts.of(TunerConstantsLastYear.FrontLeft.DriveFrictionVoltage),
                                Volts.of(TunerConstantsLastYear.FrontLeft.SteerFrictionVoltage),
                                Meters.of(TunerConstantsLastYear.FrontLeft.WheelRadius),
                                KilogramSquareMeters.of(TunerConstantsLastYear.FrontLeft.SteerInertia),
                                WHEEL_COF));
            }
            default -> {
                DRIVE_BASE_RADIUS =
                    Math.max(
                        Math.max(
                            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
                        Math.max(
                            Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                            Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
                DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
                TURN_MOTOR = DCMotor.getKrakenX60Foc(1);
                ROBOT_MASS_KG = Units.lbsToKilograms(108.4 + 11.4 + 14);
                ROBOT_MOI = (1.0 / 12.0) * ROBOT_MASS_KG * (2 * Math.pow(Units.inchesToMeters(25), 2));
                WHEEL_COF = COTS.WHEELS.COLSONS.cof;
                PP_CONFIG =
                    new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                            TunerConstants.FrontLeft.WheelRadius,
                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DRIVE_MOTOR
                                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            TunerConstants.FrontLeft.SlipCurrent,
                            1),
                        Objects.requireNonNull(getModuleTranslations()));
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(COTS.ofMark4i(
                            DRIVE_MOTOR,
                            TURN_MOTOR,
                            COTS.WHEELS.COLSONS.cof,
                            2))
                        .withTrackLengthTrackWidth(Inches.of(27.5), Inches.of(27.5))
                        .withBumperSize(Inches.of(33.5), Inches.of(33.5));
            }
        }

        if (RobotBase.isSimulation() || !org.steelhawks.RobotConfig.getConfig().hasSwerve) {
            if (!org.steelhawks.RobotConfig.getConfig().hasSwerve) {
                SimulatedArena.ALLOW_CREATION_ON_REAL_ROBOT = true;
            }
            DRIVE_SIMULATION = new SwerveDriveSimulation(MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d()));
            SimulatedArena.getInstance().addDriveTrainSimulation(DRIVE_SIMULATION);
        } else {
            DRIVE_SIMULATION = null;
        }

        Logger.recordOutput("Swerve/DriveBaseRadius", DRIVE_BASE_RADIUS);
        Logger.recordOutput("Swerve/MomentOfInertia", ROBOT_MOI);
    }

    public static SwerveDriveSimulation getDriveSimulation() {
        return DRIVE_SIMULATION;
    }

    public void updatePhysicsSimulation() {
        SimulatedArena.getInstance().simulationPeriodic();

        Pose3d[] fuelPoses =
            SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel");
        Logger.recordOutput("FieldSimulation/Fuel", fuelPoses);
        Logger.recordOutput("FieldSimulation/RobotPosition", DRIVE_SIMULATION.getSimulatedDriveTrainPose());
    }

    public void resetSimulation(Pose2d startingPose) {
        if (Constants.getMode() != Mode.SIM) return;

        DRIVE_SIMULATION.setSimulationWorldPose(startingPose);
        setPose(startingPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public Swerve(
        CANBus bus,
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        // Initialize kinematics
        this.kinematics = new SwerveDriveKinematics(
            Objects.requireNonNull(getModuleTranslations())
        );
        this.lastModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        // Initialize swerve modules
        swerveModules[0] = new SwerveModule(flModuleIO, 0,
            switch (Constants.getRobot()) {
                case OMEGABOT, SIMBOT, TEST_BOARD -> TunerConstants.FrontLeft;
                case ALPHABOT -> TunerConstantsAlpha.FrontLeft;
                case CHASSIS -> TunerConstantsChassis.FrontLeft;
                case LAST_YEAR -> TunerConstantsLastYear.FrontLeft;
            });
        swerveModules[1] = new SwerveModule(frModuleIO, 1,
            switch (Constants.getRobot()) {
                case OMEGABOT, SIMBOT, TEST_BOARD -> TunerConstants.FrontRight;
                case ALPHABOT -> TunerConstantsAlpha.FrontRight;
                case CHASSIS -> TunerConstantsChassis.FrontRight;
                case LAST_YEAR -> TunerConstantsLastYear.FrontRight;
            });
        swerveModules[2] = new SwerveModule(blModuleIO, 2,
            switch (Constants.getRobot()) {
                case OMEGABOT, SIMBOT, TEST_BOARD -> TunerConstants.BackLeft;
                case ALPHABOT -> TunerConstantsAlpha.BackLeft;
                case CHASSIS -> TunerConstantsChassis.BackLeft;
                case LAST_YEAR -> TunerConstantsLastYear.BackLeft;
            });
        swerveModules[3] = new SwerveModule(brModuleIO, 3,
            switch (Constants.getRobot()) {
                case OMEGABOT, SIMBOT, TEST_BOARD -> TunerConstants.BackRight;
                case ALPHABOT -> TunerConstantsAlpha.BackRight;
                case CHASSIS -> TunerConstantsChassis.BackRight;
                case LAST_YEAR -> TunerConstantsLastYear.BackRight;
            });
        PhoenixOdometryThread.getInstance().start(bus);

        // Configure AutoBuilder to use RobotState
        AutoBuilder.configure(
            robotState::getEstimatedPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
                new PIDConstants(
                    AutonConstants.TRANSLATION_KP.get(),
                    AutonConstants.TRANSLATION_KI.get(),
                    AutonConstants.TRANSLATION_KD.get()),
                new PIDConstants(
                    AutonConstants.ROTATION_KP.get(),
                    AutonConstants.ROTATION_KI.get(),
                    AutonConstants.ROTATION_KD.get())),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                FieldConstants.FIELD_2D.getObject("Path").setPoses(activePath);
                Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Initialize SysId routines
        driveSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, null, null,
                    (state) -> Logger.recordOutput("Swerve/DriveSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));

        turnSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, null, null,
                    (state) -> Logger.recordOutput("Swerve/TurnSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));

        angularSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null, null, null,
                    (state) -> Logger.recordOutput("Swerve/AngularSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runAngularCharacterization(voltage.in(Volts)), null, this));

        mAlignController =
            new ProfiledPIDController(
                AutonConstants.ANGLE_KP.get(),
                AutonConstants.ANGLE_KI.get(),
                AutonConstants.ANGLE_KD.get(),
                new TrapezoidProfile.Constraints(
                    AutonConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    AutonConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED));
        mAlignController.enableContinuousInput(-Math.PI, Math.PI);
        mAlignController.setTolerance(Units.degreesToRadians(3));
        mAlignDebouncer = new Debouncer(0.5, DebounceType.kRising);
        collisionDebouncer = new Debouncer(0.2, DebounceType.kRising);
        bumpRisingDebouncer = new Debouncer(0.25, DebounceType.kRising); // slow to confirm
        bumpFallingDebouncer = new Debouncer(0.05, DebounceType.kFalling); // fast to detect landing.

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        Logger.recordOutput("Swerve/Auto/Setpoint", new Pose2d());
        Logger.recordOutput("Swerve/Auto/Speeds", new ChassisSpeeds());
        Logger.recordOutput("Swerve/Auto/PID", new ChassisSpeeds());
        Logger.recordOutput("Swerve/Auto/Feedforward", new ChassisSpeeds());
    }

    public ProfiledPIDController getAlign() {
        return mAlignController;
    }

    @AutoLogOutput(key = "Swerve/AlignAtGoal")
    public boolean alignAtGoal() {
        double goal = mAlignController.getGoal().position;
        double angleDifference = MathUtil.angleModulus(goal - RobotState.getInstance().getEstimatedPose().getRotation().getRadians());
        return mAlignDebouncer.calculate(Math.abs(angleDifference) <= Units.degreesToRadians(5));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (var module : swerveModules) {
            module.periodic();
        }
        odometryLock.unlock();

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            mAlignController.setPID(
                AutonConstants.ANGLE_KP.get(),
                AutonConstants.ANGLE_KI.get(),
                AutonConstants.ANGLE_KD.get());
        }, AutonConstants.ANGLE_KP, AutonConstants.ANGLE_KI, AutonConstants.ANGLE_KD);

        if (DriverStation.isDisabled()) {
            for (var module : swerveModules) {
                module.stop();
            }
            Logger.recordOutput("SwerveStates/Setpoints", EMPTY_MODULE_STATES);
            Logger.recordOutput("SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
        }

        processOdometryObservations();
        robotState.updateChassisSpeeds(getChassisSpeeds());
        FieldConstants.FIELD_2D.setRobotPose(RobotState.getInstance().getEstimatedPose());
//        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
        LoopTimeUtil.record("Swerve");
    }

    /**
     * Process odometry observations and report to RobotState.
     */
    private void processOdometryObservations() {
        double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++) {
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                SwerveModulePosition pos = swerveModules[moduleIndex].getOdometryPositions()[i];
                // Fill deltas buffer in-place
                odometryDeltasBuffer[moduleIndex].distanceMeters =
                    pos.distanceMeters - lastModulePositions[moduleIndex].distanceMeters;
                odometryDeltasBuffer[moduleIndex].angle = pos.angle;
                // Fill positions buffer in-place (consumed synchronously in addOdometryObservation)
                odometryPositionsBuffer[moduleIndex].distanceMeters = pos.distanceMeters;
                odometryPositionsBuffer[moduleIndex].angle = pos.angle;
                lastModulePositions[moduleIndex].distanceMeters = pos.distanceMeters;
                lastModulePositions[moduleIndex].angle = pos.angle;
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Fall back to kinematics
                Twist2d twist = kinematics.toTwist2d(odometryDeltasBuffer);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            robotState.addOdometryObservation(
                new RobotState.OdometryObservation(
                    sampleTimestamps[i],
                    odometryPositionsBuffer,
                    gyroInputs.connected ? rawGyroRotation : null
                )
            );
        }
    }

    public void followTrajectory(SwerveSample sample) {
        var robot = RobotState.getInstance().getEstimatedPose();
        var nextSetpoint = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
        var speeds = autonController.getOutput(robot, nextSetpoint)
            .plus(new ChassisSpeeds(sample.vx, sample.vy, sample.omega));
        Logger.recordOutput("Swerve/Auto/Setpoint", nextSetpoint);
        Logger.recordOutput("Swerve/Auto/Speeds", speeds);
        Logger.recordOutput("Swerve/Auto/PID", speeds.minus(new ChassisSpeeds(sample.vx, sample.vy, sample.omega)));
        Logger.recordOutput("Swerve/Auto/Feedforward", new ChassisSpeeds(sample.vx, sample.vy, sample.omega));
        runVelocity(speeds);
    }

    public void followTrajectory(SwerveSample sample, boolean flipY) {
        SwerveSample flipped = sample;
        if (flipY) {
            double flippedY = FieldConstants.FIELD_WIDTH - sample.y;
            double flippedHeading = Math.PI - sample.heading;
            double flippedVy = -sample.vy;
            double flippedOmega = -sample.omega;

            flipped = new SwerveSample(
                sample.t,
                sample.x, flippedY,
                flippedHeading,
                sample.vx, flippedVy,
                sample.ax, sample.ay,
                flippedOmega, sample.alpha,
                sample.moduleForcesX(),
                sample.moduleForcesY() // might need negating too, check SwerveSample API
            );
        }
        followTrajectory(flipped);
    }

    public void runVelocity(ChassisSpeeds speeds) {
        final double speedMetersPerSec = getMaxLinearSpeedMetersPerSec();

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, speedMetersPerSec);
        currentSetpoint = discreteSpeeds;

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for (int i = 0; i < 4; i++) {
            swerveModules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runDriveCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    public void runTurnCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runTurnCharacterization(output);
        }
    }

    public void runAngularCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runAngularCharacterization(output);
        }
    }

    /** Calculates the drive wheel coefficient of static friction. */
    public double runWheelCOFCharacterizer() {
        double totalFrictionForce =
            (DRIVE_MOTOR.getTorque(TunerConstants.FrontLeft.SlipCurrent)
                * TunerConstants.FrontLeft.DriveMotorGearRatio
                / TunerConstants.FrontLeft.WheelRadius)
                * 4;
        double cof = totalFrictionForce / (ROBOT_MASS_KG * 9.81);
        Logger.recordOutput("Swerve/Characterizer/DriveWheelCOF", cof);
        System.out.println("Drive COF: " + cof);
        return cof;
    }

    public record MotorMax(double speed, double torque) {}

    public MotorMax calculateMotorMaxSpeedAndTorque() {
        // leave headroom for inefficiency and pid
        final double headroom = 0.80;
        double maxSpeed =
            Units.radiansPerSecondToRotationsPerMinute(
                DRIVE_MOTOR.freeSpeedRadPerSec * headroom);
        double maxTorque =
            DRIVE_MOTOR.getTorque(TunerConstants.FrontLeft.SlipCurrent) * headroom;
        System.out.println(
            "Motor Max Speed (rpm): " + maxSpeed + ", Motor Max Torque: " + maxTorque);
        Logger.recordOutput("Swerve/Characterizer/MaxSpeed", maxSpeed);
        Logger.recordOutput("Swerve/Characterizer/MaxTorque", maxTorque);
        return new MotorMax(maxSpeed, maxTorque);
    }

    public double calculateMotorMaxSpeed() {
        return calculateMotorMaxSpeedAndTorque().speed;
    }

    public double calculateMotorMaxTorque() {
        return calculateMotorMaxSpeedAndTorque().torque;
    }

    /** Returns the distance traveled by each individual drive wheel in radians */
    private double[] getWheelDistancesRadians() {
        SwerveModulePosition[] positions = getModulePositions();
        double[] distances = new double[4];
        for (int i = 0; i < 4; i++) {
            distances[i] = positions[i].distanceMeters / TunerConstants.FrontLeft.WheelRadius;
        }
        return distances;
    }

    public Rotation2d getPitch() {
        return gyroInputs.pitchPosition;
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = Objects.requireNonNull(getModuleTranslations())[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public double getRobotRelativeXAccelGs() {
        return gyroInputs.accelerationXInGs;
    }

    public double getRobotRelativeYAccelGs() {
        return gyroInputs.accelerationYInGs;
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        for (int i = 0; i < 4; i++) {
            moduleStatesBuffer[i].speedMetersPerSecond = swerveModules[i].getVelocityMetersPerSec();
            moduleStatesBuffer[i].angle = swerveModules[i].getAngle();
        }
        return moduleStatesBuffer;
    }

    private SwerveModulePosition[] getModulePositions() {
        for (int i = 0; i < 4; i++) {
            modulePositionsBuffer[i].distanceMeters = swerveModules[i].getPositionMeters();
            modulePositionsBuffer[i].angle = swerveModules[i].getAngle();
        }
        return modulePositionsBuffer;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Swerve/Is Slow Mode")
    public boolean isSlowMode() {
        return requestSlowMode;
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    public boolean shouldContinuePathfinding(BooleanSupplier stopCondition) {
        Logger.recordOutput("Swerve/InterruptPathfinding", stopCondition.getAsBoolean());
        return !stopCondition.getAsBoolean();
    }

    /**
     * Reset pose in both Swerve and RobotState
     */
    public void setPose(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            DRIVE_SIMULATION.setSimulationWorldPose(pose);
        }
        robotState.resetPose(pose, rawGyroRotation, getModulePositions());
        if (RobotContainer.s_ObjVision != null) {
            RobotContainer.s_ObjVision.reset();
        }
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, SIMBOT -> TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            case ALPHABOT -> TunerConstantsAlpha.kSpeedAt12Volts.in(MetersPerSecond);
            case CHASSIS -> TunerConstantsChassis.kSpeedAt12Volts.in(MetersPerSecond);
            case LAST_YEAR -> TunerConstantsLastYear.kSpeedAt12Volts.in(MetersPerSecond);
            default -> TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        };
    }

    public double getSpeedMultiplier() {
        return (requestSlowMode ? SLOW_SPEED_MULTIPLIER : SPEED_MULTIPLIER);
    }

    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    public static Translation2d[] getModuleTranslations() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, SIMBOT, TEST_BOARD ->
                new Translation2d[]{
                    new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                    new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
                };
            case ALPHABOT -> new Translation2d[]{
                new Translation2d(TunerConstantsAlpha.FrontLeft.LocationX, TunerConstantsAlpha.FrontLeft.LocationY),
                new Translation2d(TunerConstantsAlpha.FrontRight.LocationX, TunerConstantsAlpha.FrontRight.LocationY),
                new Translation2d(TunerConstantsAlpha.BackLeft.LocationX, TunerConstantsAlpha.BackLeft.LocationY),
                new Translation2d(TunerConstantsAlpha.BackRight.LocationX, TunerConstantsAlpha.BackRight.LocationY)
            };
            case CHASSIS -> new Translation2d[] {
                new Translation2d(TunerConstantsChassis.FrontLeft.LocationX, TunerConstantsChassis.FrontLeft.LocationY),
                new Translation2d(TunerConstantsChassis.FrontRight.LocationX, TunerConstantsChassis.FrontRight.LocationY),
                new Translation2d(TunerConstantsChassis.BackLeft.LocationX, TunerConstantsChassis.BackLeft.LocationY),
                new Translation2d(TunerConstantsChassis.BackRight.LocationX, TunerConstantsChassis.BackRight.LocationY)
            };
            case LAST_YEAR -> new Translation2d[]{
                new Translation2d(TunerConstantsLastYear.FrontLeft.LocationX, TunerConstantsLastYear.FrontLeft.LocationY),
                new Translation2d(TunerConstantsLastYear.FrontRight.LocationX, TunerConstantsLastYear.FrontRight.LocationY),
                new Translation2d(TunerConstantsLastYear.BackLeft.LocationX, TunerConstantsLastYear.BackLeft.LocationY),
                new Translation2d(TunerConstantsLastYear.BackRight.LocationX, TunerConstantsLastYear.BackRight.LocationY)
            };
        };
    }

    public void setPathfinding(boolean pathfinding) {
        isPathfinding = pathfinding;
    }

    public boolean isPathfinding() {
        return isPathfinding;
    }

    private boolean isCommandingHighAcceleration() {
        double dt = Timer.getFPGATimestamp() - previousSetpointTime;

        if (dt < 0.001 || previousSetpointTime == 0.0) {
            previousSetpoint = currentSetpoint;
            previousSetpointTime = Timer.getFPGATimestamp();
            return false;
        }

        double dvx = currentSetpoint.vxMetersPerSecond - previousSetpoint.vxMetersPerSecond;
        double dvy = currentSetpoint.vyMetersPerSecond - previousSetpoint.vyMetersPerSecond;

        double commandedAccelX = dvx / dt;
        double commandedAccelY = dvy / dt;
        double commandedAccelMag = Math.hypot(commandedAccelX, commandedAccelY);

        Logger.recordOutput("Swerve/Collision/CommandedAccelMagnitude", commandedAccelMag);
        previousSetpoint = currentSetpoint;
        previousSetpointTime = Timer.getFPGATimestamp();

        return commandedAccelMag > COMMANDED_ACCEL_FILTER_THRESHOLD.get();
    }

    @AutoLogOutput(key = "Swerve/Collision/Detected")
    public boolean collisionDetected() {
        double ax = gyroInputs.accelerationXInGs;
        double ay = gyroInputs.accelerationYInGs;

        double jerkX = (ax - previousAx) / Constants.UPDATE_LOOP_DT;
        double jerkY = (ay - previousAy) / Constants.UPDATE_LOOP_DT;

        previousAx = ax;
        previousAy = ay;

        double angularVelocityZ = gyroInputs.yawVelocityRadPerSec;
        double angularAccelZ = (angularVelocityZ - previousAngularVelocityZ) / Constants.UPDATE_LOOP_DT;
        double angularJerk = (angularAccelZ - previousAngularAccelZ) / Constants.UPDATE_LOOP_DT;

        previousAngularVelocityZ = angularVelocityZ;
        previousAngularAccelZ = angularAccelZ;

        double jerkMag = Math.hypot(jerkX, jerkY);
        double angularJerkMag = Math.abs(angularJerk);

        Logger.recordOutput("Swerve/Collision/JerkMagnitude", jerkMag);
        Logger.recordOutput("Swerve/Collision/AngularJerkMagnitude", angularJerkMag);

        boolean highCommandedAccel = isCommandingHighAcceleration();
        boolean linearCollision = jerkMag > COLLISION_JERK_THRESHOLD.get() && !highCommandedAccel;
        boolean angularCollision = angularJerkMag > COLLISION_ANG_JERK_THRESHOLD.get();
        return collisionDebouncer.calculate(linearCollision || angularCollision);
    }

    @AutoLogOutput(key = "Swerve/IsOnBump")
    public boolean isOnBump() {
        double tilt = Math.hypot(gyroInputs.rollPosition.getDegrees(), gyroInputs.pitchPosition.getDegrees());
        boolean rawTilt = tilt > BUMP_ANGLE_THRESHOLD.get();
        return bumpRisingDebouncer.calculate(rawTilt) || bumpFallingDebouncer.calculate(rawTilt);
    }

    // Command Factories
    public Command toggleMultiplier() {
        return Commands.runOnce(() -> requestSlowMode = !requestSlowMode);
    }

    public Command toggleLowGear() {
        return Commands.runOnce(() -> requestSlowMode = true);
    }

    public Command toggleNormal() {
        return Commands.runOnce(() -> requestSlowMode = false);
    }

    public Command zeroHeading() {
        return Commands.runOnce(
            () -> {
                Pose2d zeroed = new Pose2d(RobotState.getInstance().getEstimatedPose().getTranslation(), new Rotation2d());

                if (RobotBase.isSimulation()) {
                    zeroed = new Pose2d(DRIVE_SIMULATION.getSimulatedDriveTrainPose().getTranslation(), new Rotation2d());
                }

                setPose(zeroed);
            }, this)
        .ignoringDisable(true);
    }

    public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runDriveCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(driveSysId.quasistatic(direction));
    }

    public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runDriveCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(driveSysId.dynamic(direction));
    }

    public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runVelocity(new ChassisSpeeds(0.0, 0.0, .01)))
            .withTimeout(1.0)
            .andThen(turnSysId.quasistatic(direction));
    }

    public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runVelocity(new ChassisSpeeds(0.0, 0.0, 0.01)))
            .withTimeout(1.0)
            .andThen(turnSysId.dynamic(direction));
    }

    public Command angularSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runAngularCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(angularSysId.quasistatic(direction));
    }

    public Command angularSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runAngularCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(angularSysId.dynamic(direction));
    }

    public Command testZeroedModules() {
        return Commands.run(() -> runDriveCharacterization(0.0), this);
    }

    public boolean isStalling() {
        for (int i = 0; i < 4; i++) {
            if (swerveModules[i].getIsStalling()) {
                return true;
            }
        }
        return false;
    }
}