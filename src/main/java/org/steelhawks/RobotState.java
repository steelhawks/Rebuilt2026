package org.steelhawks;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LatchedBoolean;
import org.steelhawks.util.VirtualSubsystem;
import org.steelhawks.util.geometry.Boundary;
import org.steelhawks.util.geometry.RobotFootprint;

import java.util.*;

public class RobotState extends VirtualSubsystem {

    private final RobotFootprint footprint =
        new RobotFootprint(
            Constants.RobotConstants.ROBOT_LENGTH_WITH_BUMPERS,
            Constants.RobotConstants.ROBOT_WIDTH_WITH_BUMPERS)
                .withExtension(new RobotFootprint.Extension(
                "Intake",
                    Rotation2d.fromDegrees(0.0),
                () -> RobotContainer.s_Intake == null ? 0.0 : RobotContainer.s_Intake.getPosition()));

    private static final double movingVelocityThreshold = 0.5; // m/s
    private static final double poseBufferSizeSec = 2.0;
    private static final double intakeExtensionBufferSizeSec = 2.0;
    private static final double objectMaxAgeSec = 1.0;

    public enum ShiftState {
        AUTO(20.0),
        TRANSITION(10.0),
        SHIFT1(25.0),
        SHIFT2(25.0),
        SHIFT3(25.0),
        SHIFT4(25.0),
        END_GAME(30.0);

        private double time;

        ShiftState(double time) {
            this.time = time;
        }
    }

    public enum ShooterMode {
        TO_HUB,
        FERRY,
        MANUAL
    }

    public enum ShootingState {
        SHOOTING_STATIONARY,
        SHOOTING_MOVING,
        SHOOTING, // used to signify that we are just shooting, the set function will automatically decide if we are sotm or stationary
        NOTHING
    }

    // Triggers
    @AutoLogOutput
    private final Trigger sotmTrigger;
    @AutoLogOutput
    private final Trigger inTrenchTrigger;
    @AutoLogOutput
    private final Trigger inBumpTrigger;

    private ShootingState lastDerivedShootingState = ShootingState.SHOOTING_STATIONARY;
    private ShooterMode currentShooterMode = ShooterMode.TO_HUB;
    private ShootingState shootingState = ShootingState.NOTHING;
    private ShiftState shiftState = ShiftState.AUTO;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final TimeInterpolatableBuffer<Translation2d> intakeExtensionBuffer =
        TimeInterpolatableBuffer.createBuffer(intakeExtensionBufferSizeSec);

    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private Rotation2d gyroRotation = new Rotation2d();
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(Objects.requireNonNull(Swerve.getModuleTranslations()));
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    private final SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final SwerveDriveOdometry wheelOdometry =
        new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private final List<TimestampedObjectList> objectHistory;
    private List<DetectedObject> currentDetectedObjects;

    // Goal Tracking
    private final LatchedBoolean teleopStarted = new LatchedBoolean();
    private final LatchedBoolean autoStarted = new LatchedBoolean();
    private Alliance initialActiveHub = null;
    private Alliance activeHub = null;

    private final Timer timer = new Timer();
    private static RobotState instance;

    private RobotState() {
        this.objectHistory = new ArrayList<>();
        this.currentDetectedObjects = new ArrayList<>();

        sotmTrigger = new Trigger(
            () -> getAimState().equals(ShootingState.SHOOTING_MOVING));
        inTrenchTrigger =
            Boundary.asTrigger(
                "LeftTrench",
                () -> AllianceFlip.apply(FieldConstants.Trench.TRENCH_LEFT_TRIGGER_BOX),
                this::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER)
            .or(Boundary.asTrigger(
                "RightTrench",
                () -> AllianceFlip.apply(FieldConstants.Trench.TRENCH_RIGHT_TRIGGER_BOX),
                this::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER))
            .debounce(0.3);

        inBumpTrigger =
            Boundary.asTrigger(
                () -> AllianceFlip.apply(new Rectangle2d(new Translation2d(), new Translation2d())),
                this::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER)
            .debounce(0.3);
    }

    public RobotFootprint getFootprint() {
        return footprint;
    }

    public Trigger getTrenchTrigger() {
        return inTrenchTrigger;
    }

    public Trigger getBumpTrigger() {
        return inBumpTrigger;
    }

    public Trigger getSOTMTrigger() {
        return sotmTrigger;
    }

    public void updateChassisSpeeds(ChassisSpeeds speeds) {
        this.currentChassisSpeeds = speeds;
    }

    public void setShooterMode(ShooterMode mode) {
        if (currentShooterMode != mode) {
            Logger.recordOutput("ShooterMode/ModeChange",
                currentShooterMode.name() + " -> " + mode.name());
            currentShooterMode = mode;
            Logger.recordOutput("ShooterMode/CurrentMode", mode.name());
            // clear trajectory
            Logger.recordOutput("Turret/ScoreTrajectory", new Translation3d[0]);
            Logger.recordOutput("Turret/FerryTrajectory", new Translation3d[0]);
        }
    }

    public void setAimState(ShootingState state) {
        if (shootingState != state) {
            Logger.recordOutput("AimState/ModeChange",
                shootingState.name() + " -> " + state.name());
            shootingState = state;
            Logger.recordOutput("AimState/CurrentMode", state.name());
            if (state != ShootingState.SHOOTING) {
                lastDerivedShootingState = ShootingState.SHOOTING_STATIONARY;
            }
        }
    }

    public ShootingState getAimState() {
        if (shootingState == ShootingState.NOTHING) {
            return ShootingState.NOTHING;
        }
        double linearVelocity = Math.hypot(
            currentChassisSpeeds.vxMetersPerSecond,
            currentChassisSpeeds.vyMetersPerSecond);
        if (shootingState == ShootingState.SHOOTING) {
            double threshold = lastDerivedShootingState == ShootingState.SHOOTING_MOVING
                ? movingVelocityThreshold * 0.5
                : movingVelocityThreshold;
            lastDerivedShootingState = linearVelocity > threshold
                ? ShootingState.SHOOTING_MOVING
                : ShootingState.SHOOTING_STATIONARY;
            return lastDerivedShootingState;
        }
        return shootingState;
    }

    public ShooterMode getShooterMode() {
        return currentShooterMode;
    }

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    @Override
    public void periodic() {
//        if (autoStarted.update(DriverStation.isAutonomous())) {
//            shiftState = ShiftState.AUTO;
//            initialActiveHub = null;
//            activeHub = null;
//            timer.stop();
//            timer.reset();
//            Logger.recordOutput("RobotState/ShiftState", shiftState.name());
//        }
//        if (teleopStarted.update(DriverStation.isTeleop())) {
//            shiftState = ShiftState.TRANSITION;
//            String gameData = DriverStation.getGameSpecificMessage();
//            if (gameData.isEmpty()) {
//                initialActiveHub = Alliance.Blue;
//            } else {
//                initialActiveHub = (gameData.charAt(0) == 'B') ? Alliance.Red : Alliance.Blue;
//                Logger.recordOutput("RobotState/GameData", gameData);
//            }
//            activeHub = initialActiveHub;
//            Logger.recordOutput("RobotState/InitialActiveHub", initialActiveHub.name());
//            timer.restart();
//            Logger.recordOutput("RobotState/ShiftState", shiftState.name());
//        }
//
//        if (DriverStation.isTeleop() && timer.isRunning()) {
//            if (timer.advanceIfElapsed(shiftState.time)) {
//                if (shiftState != ShiftState.END_GAME) {
//                    shiftState = ShiftState.values()[shiftState.ordinal() + 1];
//                    if (shiftState == ShiftState.SHIFT1 ||
//                        shiftState == ShiftState.SHIFT2 ||
//                        shiftState == ShiftState.SHIFT3 ||
//                        shiftState == ShiftState.SHIFT4) {
//                        activeHub = (activeHub == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
//                    }
//                    Logger.recordOutput("RobotState/ShiftState", shiftState.name());
//                    Logger.recordOutput("RobotState/ActiveHub", activeHub != null ? activeHub.name() : "BOTH");
//                }
//            }
//        }
//        if (currentShooterMode == ShooterMode.MANUAL) {
//            return;
//        }
//        ShooterMode desiredMode = calculateDesiredMode();
//        if (desiredMode != currentShooterMode) {
//            setShooterMode(desiredMode);
//        }
    }

    private ShooterMode calculateDesiredMode() {
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty()) {
            return ShooterMode.TO_HUB;
        }
        boolean inAllianceZone = isInAllianceZone(ourAlliance.get());
        if (inAllianceZone || areBothHubsActive()) {
            return ShooterMode.TO_HUB;
        }
        if (isOurHubActive(ourAlliance.get())) {
            return ShooterMode.TO_HUB;
        } else {
            return ShooterMode.FERRY;
        }
    }

    private boolean isInAllianceZone(Alliance ourAlliance) {
        Pose2d robotPose = getEstimatedPose();
        double x = robotPose.getX();
        double allianceZoneDepth = AllianceFlip.applyX(Units.inchesToMeters(158.6));
        return ourAlliance == Alliance.Blue
            ? x <= allianceZoneDepth
            : x >= allianceZoneDepth;
    }

    private boolean areBothHubsActive() {
        return shiftState == ShiftState.AUTO ||
            shiftState == ShiftState.TRANSITION ||
            shiftState == ShiftState.END_GAME;
    }

    private boolean isOurHubActive(Alliance ourAlliance) {
        return activeHub == null || activeHub == ourAlliance;
    }

    /**
     * Returns which alliance's hub is currently active during alternating shifts.
     * Returns empty if both hubs are active (AUTO, TRANSITION, END_GAME).
     */
    public Optional<Alliance> getActiveHub() {
        if (areBothHubsActive()) {
            return Optional.empty();
        }
        return Optional.ofNullable(activeHub);
    }

    /**
     * Returns true if our alliance's hub is currently active.
     */
    public boolean isOurHubActive() {
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty() || areBothHubsActive()) {
            return true;
        }
        return isOurHubActive(ourAlliance.get());
    }

    /**
     * Returns the current shift state.
     */
    public ShiftState getShiftState() {
        return shiftState;
    }

    public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        gyroRotation = gyroAngle;
        rawGyroRotation = gyroAngle;
        poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
        wheelOdometry.resetPosition(gyroAngle, modulePositions, pose);

        // clear and reinit buffers
        poseBuffer.clear();
        poseBuffer.addSample(Timer.getFPGATimestamp(), pose);
        intakeExtensionBuffer.clear();
        objectHistory.clear();
        Logger.recordOutput("RobotState/PoseReset", pose);
    }

    public void addOdometryObservation(OdometryObservation observation) {
        if (observation.gyroAngle().isPresent()) {
            gyroRotation = observation.gyroAngle().get();
            rawGyroRotation = observation.gyroAngle().get();
        }
        Pose2d estimatedPose = poseEstimator.updateWithTime(
            observation.timestamp(), gyroRotation, observation.wheelPositions());
        Pose2d wheelOnlyPose = wheelOdometry.update(
            gyroRotation, observation.wheelPositions());
        poseBuffer.addSample(observation.timestamp(), estimatedPose);
        Logger.recordOutput("RobotState/EstimatedPose", estimatedPose);
        Logger.recordOutput("RobotState/WheelOdometryPose", wheelOnlyPose);
    }

    public void addVisionObservation(VisionObservation observation) {
        poseEstimator.addVisionMeasurement(
            observation.robotPose(),
            observation.timestamp(),
            observation.stdDevs()
        );
        Logger.recordOutput("RobotState/LatestVisionPose", observation.robotPose());
    }

    public void addObjectDetections(List<DetectedObject> objects, double timestamp) {
        currentDetectedObjects = new ArrayList<>(objects);

        objectHistory.add(new TimestampedObjectList(timestamp, new ArrayList<>(objects)));
        double now = Timer.getFPGATimestamp();
        objectHistory.removeIf(entry -> now - entry.timestamp > objectMaxAgeSec);
        Pose3d[] objectPoses = objects.stream()
            .map(DetectedObject::pose)
            .toArray(Pose3d[]::new);
        Logger.recordOutput("RobotState/DetectedObjects", objectPoses);
        Logger.recordOutput("RobotState/DetectedObjectCount", objects.size());
    }

    public void addIntakeExtension(double distanceMeters, double timestamp) {
        intakeExtensionBuffer.addSample(timestamp, new Translation2d(distanceMeters, 0.0));
    }

    @AutoLogOutput(key = "RobotState/PoseEstimation/PoseEstimation")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getEstimatedPose().getRotation();
    }

    @AutoLogOutput(key = "RobotState/PoseEstimation/Odometry")
    public Pose2d getWheelOdometryPose() {
        return wheelOdometry.getPoseMeters();
    }

    public Optional<Pose2d> getPoseAtTime(double timestamp) {
        return poseBuffer.getSample(timestamp);
    }

    /**
     * Get the turret angle at a specific timestamp
     */
    public Optional<Translation2d> getIntakeExtensionAtTime(double timestamp) {
        return intakeExtensionBuffer.getSample(timestamp);
    }

    public Optional<List<DetectedObject>> getObjectsAtTime(double timestamp) {
        if (objectHistory.isEmpty()) {
            return Optional.empty();
        }
        TimestampedObjectList closest = objectHistory.stream()
            .min(Comparator.comparingDouble(entry -> Math.abs(entry.timestamp - timestamp)))
            .orElse(null);
        if (Math.abs(closest.timestamp - timestamp) > objectMaxAgeSec) {
            return Optional.empty();
        }
        return Optional.of(new ArrayList<>(closest.objects));
    }

    public List<DetectedObject> getDetectedObjects() {
        return new ArrayList<>(currentDetectedObjects);
    }

    public Optional<DetectedObject> getClosestObject() {
        return getClosestObject(currentDetectedObjects);
    }

    public Optional<DetectedObject> getClosestObject(List<DetectedObject> objects) {
        if (objects.isEmpty()) {
            return Optional.empty();
        }

        Pose2d robotPose = getEstimatedPose();
        return objects.stream()
            .min((o1, o2) -> {
                double dist1 = robotPose.getTranslation()
                    .getDistance(o1.pose().toPose2d().getTranslation());
                double dist2 = robotPose.getTranslation()
                    .getDistance(o2.pose().toPose2d().getTranslation());
                return Double.compare(dist1, dist2);
            });
    }

    public List<DetectedObject> getObjectsInRadius(double radiusMeters) {
        Pose2d robotPose = getEstimatedPose();
        return currentDetectedObjects.stream()
            .filter(obj -> {
                double distance = robotPose.getTranslation()
                    .getDistance(obj.pose().toPose2d().getTranslation());
                return distance <= radiusMeters;
            })
            .toList();
    }

    private record TimestampedObjectList(double timestamp, List<DetectedObject> objects) {}

    public record OdometryObservation(
        double timestamp, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle) {}

    public record VisionObservation(
        double timestamp,
        Pose2d robotPose,
        Vector<N3> stdDevs) {}

    /** Represents a robot pose sample used for pose estimation. */
    public record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type) {}

    public enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    public record DetectedObject(
        Pose3d pose,
        String type,
        double confidence,
        double timestamp
    ) {}
}
