package org.steelhawks.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.steelhawks.Constants.*;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.Toggles;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsLastYear;
import org.steelhawks.util.LocalADStarAK;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.LoopTimeUtil;

public class Swerve extends SubsystemBase {

    private static final double SLOW_SPEED_MULTIPLIER = 0.3;
    private static final double SPEED_MULTIPLIER = 1.0;
    private boolean isPathfinding = false;
    private boolean requestSlowMode = false;

    public static final double ODOMETRY_FREQUENCY =
        Constants.getCANBus().isNetworkFD() ? 250.0 : 100.0;
    private static final double POSE_BUFFER_SIZE_SEC = 2.0;
    public static final double DRIVE_BASE_RADIUS;

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG;
    private static final double ROBOT_MOI;
    private static final double WHEEL_COF;
    private static final RobotConfig PP_CONFIG;

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG;
    private static final SwerveDriveSimulation DRIVE_SIMULATION;

    private final LoggedTunableNumber COLLISION_ACCEL_THRESHOLD = new LoggedTunableNumber("Swerve/CollisionAccelThreshold", 0.0);
    private double previousAx = 0.0;
    private double previousAy = 0.0;

    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR
    private final SysIdRoutine driveSysId;
    private final SysIdRoutine turnSysId;
    private final SysIdRoutine angularSysId;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    private final SwerveDrivePoseEstimator mPoseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);

    private final ProfiledPIDController mAlignController;
    private final Debouncer mAlignDebouncer;
    private final Debouncer collisionDebouncer;

    static {
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
                            DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstantsAlpha.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsAlpha.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getFalcon500(1),
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
                            DCMotor.getKrakenX60Foc(1)
                                .withReduction(TunerConstantsLastYear.FrontLeft.DriveMotorGearRatio),
                            TunerConstantsLastYear.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                DCMotor.getKrakenX60(1),
                                DCMotor.getFalcon500(1),
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
                            DCMotor.getKrakenX60(1)
                                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            TunerConstants.FrontLeft.SlipCurrent,
                            1),
                        getModuleTranslations());
                MAPLE_SIM_CONFIG =
                    DriveTrainSimulationConfig.Default()
                    // Specify gyro type (for realistic gyro drifting and error simulation)
                    .withGyro(COTS.ofPigeon2())
                    // Specify swerve module (for realistic swerve dynamics)
                    .withSwerveModule(COTS.ofMark4n(
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getKrakenX60Foc(1),
                        COTS.WHEELS.COLSONS.cof,
                        3)) // L3 Gear ratio
                    // Configures the track length and track width (spacing between swerve modules)
                    .withTrackLengthTrackWidth(Inches.of(25), Inches.of(25))
                    // Configures the bumper size (dimensions of the robot bumper)
                    .withBumperSize(Inches.of(36), Inches.of(36));
            }
        }

        if (RobotBase.isSimulation()) {
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
        if (Constants.getMode() != Mode.SIM) return;

        // physics sim to simulate the field
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
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        swerveModules[0] = new SwerveModule(flModuleIO, 0,
            switch (Constants.getRobot()) {
                case ALPHABOT -> TunerConstantsAlpha.FrontLeft;
                case LAST_YEAR -> TunerConstantsLastYear.FrontLeft;
                default -> TunerConstants.FrontLeft;
            });
        swerveModules[1] = new SwerveModule(frModuleIO, 1,
            switch (Constants.getRobot()) {
                case ALPHABOT -> TunerConstantsAlpha.FrontRight;
                case LAST_YEAR -> TunerConstantsLastYear.FrontRight;
                default -> TunerConstants.FrontRight;
            });
        swerveModules[2] = new SwerveModule(blModuleIO, 2,
            switch (Constants.getRobot()) {
                case ALPHABOT -> TunerConstantsAlpha.BackLeft;
                case LAST_YEAR -> TunerConstantsLastYear.BackLeft;
                default -> TunerConstants.BackLeft;
            });
        swerveModules[3] = new SwerveModule(brModuleIO, 3,
            switch (Constants.getRobot()) {
                case ALPHABOT -> TunerConstantsAlpha.BackRight;
                case LAST_YEAR -> TunerConstantsLastYear.BackRight;
                default -> TunerConstants.BackRight;
            });

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPose,
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
                Logger.recordOutput(
                    "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        driveSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Swerve/DriveSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));

        turnSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Swerve/TurnSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));

        angularSysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
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

        // warm up pathplanner lib
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    public ProfiledPIDController getAlign() {
        return mAlignController;
    }

    @AutoLogOutput(key = "Swerve/AlignAtGoal")
    public boolean alignAtGoal() {
        double goal = mAlignController.getGoal().position;
        double angleDifference = MathUtil.angleModulus(goal - getPose().getRotation().getRadians());
        return mAlignDebouncer.calculate(Math.abs(angleDifference) <= Units.degreesToRadians(5));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
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

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : swerveModules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // Update odometry
        double[] sampleTimestamps =
            swerveModules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            mPoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
            Pose2d odometryOnlyPose =
                odometry.update(rawGyroRotation, modulePositions);
            poseBuffer.addSample(sampleTimestamps[i], odometryOnlyPose);
        }

        FieldConstants.FIELD_2D.setRobotPose(getPose());
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
        LoopTimeUtil.record("Swerve");
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        final double speedMetersPerSec =
            switch (Constants.getRobot()) {
                case ALPHABOT -> TunerConstantsAlpha.kSpeedAt12Volts.in(MetersPerSecond);
                case LAST_YEAR -> TunerConstantsLastYear.kSpeedAt12Volts.in(MetersPerSecond);
                default -> TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
            };

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, speedMetersPerSec);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for (int i = 0; i < 4; i++) {
            swerveModules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Accepts a vision measurement and updates the pose estimator.
     *
     * @param visionRobotPoseMeters The robot pose measurement from the vision system.
     * @param timestampSeconds The timestamp of the vision measurement.
     * @param visionMeasurementStdDevs The standard deviations of the vision measurement.
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {

        if (!Toggles.Vision.visionEnabled.get()) return;

        mPoseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     */
    public void runDriveCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runCharacterization(output);
        }
    }

    /**
     * Runs the robot in a circular motion with the specified turn output.
     */
    public void runTurnCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runTurnCharacterization(output);
        }
    }

    /**
     * Runs the robot in a circular motion with the specified turn output.
     */
    public void runAngularCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].runAngularCharacterization(output);
        }
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns the robot's acceleration in the X direction relative to the robot.
     */
    public double getRobotRelativeXAccelGs() {
        return gyroInputs.accelerationXInGs;
    }

    /**
     * Returns the robot's acceleration in the Y direction relative to the robot.
     */
    public double getRobotRelativeYAccelGs() {
        return gyroInputs.accelerationYInGs;
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = swerveModules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Swerve/Is Slow Mode")
    public boolean isSlowMode() {
        return requestSlowMode;
    }

    /**
     * Returns the position of each module in radians.
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = swerveModules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += swerveModules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Odometry/RobotWheelOdom")
    public Pose2d getWheelOdomPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns a pose at a certain timestamp. Interpolates inbetween to find.
     */
    public Optional<Pose2d> getPoseAtTime(double timestamp) {
        return poseBuffer.getSample(timestamp);
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Calculates the angle the robot needs to turn in order to face
     * a specified target position. The angle is returned as a Rotation2d object
     * and is normalized to the range [-π, π] radians for easier use in rotation-based calculations.
     */
    public Rotation2d calculateTurnAngle(Pose2d targetAngle) {
        double dx = targetAngle.getX() - getPose().getX();
        double dy = targetAngle.getY() - getPose().getY();

        double angleToTarget = Math.atan2(dy, dx);
        double calculatedAngle = angleToTarget - getRotation().getRadians();
        return new Rotation2d(Math.IEEEremainder(calculatedAngle, 2 * Math.PI));
    }

    public boolean shouldContinuePathfinding(BooleanSupplier stopCondition) {
        Logger.recordOutput("Swerve/InterruptPathfinding", stopCondition.getAsBoolean());
        return !stopCondition.getAsBoolean();
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            DRIVE_SIMULATION.setSimulationWorldPose(pose);
        }
        mPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        odometry.resetPosition(rawGyroRotation, getModulePositions(), pose);

        // reset buffer b/c of discontinuous jump
        poseBuffer.clear();
        poseBuffer.addSample(Timer.getFPGATimestamp(), pose);
        if (RobotContainer.s_ObjVision != null) {
            RobotContainer.s_ObjVision.reset();
        }
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> TunerConstantsAlpha.kSpeedAt12Volts.in(MetersPerSecond);
            case LAST_YEAR -> TunerConstantsLastYear.kSpeedAt12Volts.in(MetersPerSecond);
            default -> TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        };
    }

    /**
     * Returns the speed multiplier.
     */
    public double getSpeedMultiplier() {
        return (requestSlowMode ? SLOW_SPEED_MULTIPLIER : SPEED_MULTIPLIER);
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return switch (Constants.getRobot()) {
            case OMEGABOT, SIMBOT ->
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

    public boolean collisionDetected() {
        double ax = gyroInputs.accelerationXInGs;
        double ay = gyroInputs.accelerationYInGs;

        double jerkX = ax - previousAx;
        double jerkY = ay - previousAy;

        previousAx = ax;
        previousAy = ay;

        double jerkMag = Math.abs(Math.hypot(jerkX, jerkY));
        return collisionDebouncer.calculate(jerkMag > COLLISION_ACCEL_THRESHOLD.get());
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command toggleMultiplier() {
        return Commands.runOnce(() -> requestSlowMode = !requestSlowMode);
    }

    public Command zeroHeading() {
        return Commands.runOnce(
            () -> {
                Pose2d zeroed = new Pose2d(getPose().getTranslation(), new Rotation2d());

                if (RobotBase.isSimulation()) {
                    zeroed = new Pose2d(DRIVE_SIMULATION.getSimulatedDriveTrainPose().getTranslation(), new Rotation2d());
                }

                setPose(zeroed);
            }, this)
            .ignoringDisable(true);
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runDriveCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(driveSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runDriveCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(driveSysId.dynamic(direction));
    }

    /**
     * Returns a command to run a quasistatic test for the turn motors.
     */
    public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runVelocity(
            new ChassisSpeeds(
                0.0,
                0.0,
                .01)))
            .withTimeout(1.0)
            .andThen(turnSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test for the turn motors.
     */
    public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runVelocity(
            new ChassisSpeeds(
                0.0,
                0.0,
                0.01)))
            .withTimeout(1.0)
            .andThen(turnSysId.dynamic(direction));
    }

    /**
     * Returns a command to run a quasistatic test to characterize the robot's angular motion.
     */
    public Command angularSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runAngularCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(angularSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test to characterize the robot's angular motion.
     */
    public Command angularSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runAngularCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(angularSysId.dynamic(direction));
    }

    /**
     * Returns a command that keeps the module zeroed. Useful for testing if the CANcoder offsets are valid.
     */
    public Command testZeroedModules() {
        return Commands.run(
            () -> runDriveCharacterization(0.0), this);
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
