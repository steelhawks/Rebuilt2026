package org.steelhawks;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LatchedBoolean;
import org.steelhawks.util.Maths;
import org.steelhawks.util.geometry.Boundary;
import org.steelhawks.util.geometry.RobotFootprint;

import java.util.*;

public class RobotState {

    private final RobotFootprint footprint =
        new RobotFootprint(
            Constants.RobotConstants.ROBOT_LENGTH_WITH_BUMPERS,
            Constants.RobotConstants.ROBOT_WIDTH_WITH_BUMPERS)
                .withExtension(new RobotFootprint.Extension(
                "Intake",
                    Rotation2d.fromDegrees(0.0),
                () -> Subsystems.intakeIfPresent()
                    .map(Intake::getPosition)
                    .orElse(0.0)));

    private static final double movingVelocityThreshold = 0.1; // m/s
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

    public enum AimState {
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
    @AutoLogOutput
    private final Trigger turretStuckTrigger;
    @AutoLogOutput
    private final Trigger nearHubTrigger;

    private ShooterStructure.MovingShotSolution movingShotSolution = null;

    private ShootingState lastDerivedShootingState = ShootingState.SHOOTING_STATIONARY;
    private AimState currentAimState = AimState.TO_HUB;
    private ShootingState shootingState = ShootingState.NOTHING;
    private ShiftState shiftState = ShiftState.AUTO;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    // Wheel-only history, keyed by odometry timestamp. Separate from poseBuffer
    // (which stores the *fused* estimate) because latency compensation has to
    // measure wheel motion since the Pi's sample - feeding the fused pose back
    // into itself would compound its own correction every cycle.
    private final TimeInterpolatableBuffer<Pose2d> wheelPoseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final TimeInterpolatableBuffer<Translation2d> intakeExtensionBuffer =
        TimeInterpolatableBuffer.createBuffer(intakeExtensionBufferSizeSec);

    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private double previousFieldVelocityTimestampSec = 0.0;
    private Translation2d previousFieldVelocity = new Translation2d();
    private Translation2d filteredFieldAcceleration = new Translation2d();
    // Gyro-derived linear acceleration in robot body frame, m/s^2 (gravity removed).
    // gyroBodyAccelValid is true only when the IMU is connected AND reports valid
    // linear-accel signals; otherwise SOTM falls back to the velocity-derivative path.
    private Translation2d gyroBodyLinearAccelMps2 = new Translation2d();
    private boolean gyroBodyAccelValid = false;
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

    // Wheel-only dead-reckoning pose. This runs continuously the whole match
    // and is the fallback estimate the RIO uses whenever the Pi's fused pose is
    // stale (link drop) - never frozen, so recovery is seamless.
    private final SwerveDriveOdometry wheelOdometry =
        new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    // ---- Fused pose from the Orange Pi (GTSAM), applied via PoseLink ----
    /** Staleness threshold; past this, getEstimatedPose() falls back to wheel odometry. */
    public static final double FUSED_STALENESS_TIMEOUT_SEC = 0.2;

    private Pose2d fusedPose = new Pose2d();
    private double fusedSampleTimestamp = 0.0;
    private long fusedSeqnum = -1;
    private double fusedAppliedWallClock = Double.NEGATIVE_INFINITY;

    // Latest odometry sample, handed to PoseLink to ship to the Pi.
    private OdometryObservation latestOdometry = null;

    // Pose-reset command for the Pi. resetRequestSeqnum bumps on every resetPose;
    // PoseLink forwards it until the Pi acknowledges the seqnum.
    private long resetRequestSeqnum = 0;
    private Pose2d resetRequestPose = new Pose2d();

    private final List<TimestampedObjectList> objectHistory;
    private List<DetectedObject> currentDetectedObjects;

    // Goal Tracking
    private final LatchedBoolean matchStarted = new LatchedBoolean();
    private final LatchedBoolean autoStarted = new LatchedBoolean();
    private final LatchedBoolean teleopStarted = new LatchedBoolean();
    private Alliance initialActiveHub = null;
    private Alliance activeHub = null;

    private final Timer timer = new Timer();
    private static RobotState instance;

    private RobotState() {
        this.objectHistory = new ArrayList<>();
        this.currentDetectedObjects = new ArrayList<>();

        sotmTrigger = new Trigger(
            () -> getShootingState().equals(ShootingState.SHOOTING_MOVING));
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
        nearHubTrigger =
            Boundary.asTrigger(
                "NearHub",
                () -> AllianceFlip.apply(FieldConstants.Hub.NEAR_HUB_TRIGGER_BOX),
                this::getEstimatedPose,
                footprint,
                Boundary.Mode.PERIMETER)
            .debounce(0.3);
        turretStuckTrigger =
            new Trigger(
                () -> Subsystems.turretIfPresent()
                    .map(org.steelhawks.subsystems.superstructure.turret.Turret::isJammedOrInDeadSpot)
                    .orElse(false)
                    && shootingState != ShootingState.NOTHING);
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

    public Trigger getNearHubTrigger() {
        return nearHubTrigger;
    }

    public Trigger getSOTMTrigger() {
        return sotmTrigger;
    }

    public Trigger getTurretJamTrigger() {
        return turretStuckTrigger;
    }

    public void updateChassisSpeeds(ChassisSpeeds speeds) {
        this.currentChassisSpeeds = speeds;
    }

    /** Current chassis translational velocity rotated into the field frame, in m/s. */
    public Translation2d getFieldRelativeVelocity() {
        return new Translation2d(
            currentChassisSpeeds.vxMetersPerSecond,
            currentChassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
    }

    /**
     * Pushes gravity-compensated body-frame linear acceleration from the IMU.
     * Pass null + valid=false when the source is unavailable (sim, NavX, or
     * Pigeon disconnect); SOTM will fall back to differentiating chassis velocity.
     */
    public void updateGyroAcceleration(Translation2d bodyFrameMps2, boolean valid) {
        if (bodyFrameMps2 != null) {
            this.gyroBodyLinearAccelMps2 = bodyFrameMps2;
        }
        this.gyroBodyAccelValid = valid;
    }

    public void setAimState(AimState mode) {
        if (currentAimState != mode) {
            Logger.recordOutput("AimState/ModeChange",
                currentAimState.name() + " -> " + mode.name());
            currentAimState = mode;
            Logger.recordOutput("AimState/CurrentMode", mode.name());
            // clear trajectory
            Logger.recordOutput("Turret/ScoreTrajectory", new Translation3d[0]);
            Logger.recordOutput("Turret/FerryTrajectory", new Translation3d[0]);
        }
    }

    public void setShootingState(ShootingState state) {
        if (shootingState != state) {
            Logger.recordOutput("ShooterState/ModeChange",
                shootingState.name() + " -> " + state.name());
            shootingState = state;
//            Logger.recordOutput("ShooterState/CurrentMode", state.name());
            if (state != ShootingState.SHOOTING) {
                lastDerivedShootingState = ShootingState.SHOOTING_STATIONARY;
            }
        }
    }

    @AutoLogOutput(key = "ShooterState/CurrentMode")
    public ShootingState getShootingState() {
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

    public AimState getAimState() {
        return currentAimState;
    }

    public void updateMovingShot() {
        var target =
            getAimState().equals(AimState.TO_HUB)
                ? AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D)
                : AllianceFlip.apply(
                    Maths.fromTranslation2dWithZ(
                        FieldConstants.getClosestPointOnLine(
                            FieldConstants.Ferrying.START_LINE,
                            FieldConstants.Ferrying.END_LINE),
                        0.0));
        Translation3d robotVelocity = new Translation3d(
            currentChassisSpeeds.vxMetersPerSecond,
            currentChassisSpeeds.vyMetersPerSecond,
            0.0);

        Translation2d currentFieldVelocity =
            new Translation2d(
                currentChassisSpeeds.vxMetersPerSecond,
                currentChassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());
        double now = Timer.getFPGATimestamp();
        double dt = now - previousFieldVelocityTimestampSec;

        // Pick the acceleration source. Pigeon is preferred because it measures the
        // robot's REAL body acceleration (including defense hits, wheel slip) with
        // ~1ms latency, whereas the velocity derivative only sees motion the
        // odometry can resolve. Falls back to the derivative if the IMU isn't
        // reporting valid linear-accel signals.
        Translation2d rawAccel = null;
        String accelSource;
        if (gyroBodyAccelValid) {
            rawAccel = gyroBodyLinearAccelMps2.rotateBy(getRotation());
            accelSource = "Pigeon";
        } else if (previousFieldVelocityTimestampSec > 0.0 && dt > 1e-4 && dt < 0.1) {
            rawAccel = currentFieldVelocity.minus(previousFieldVelocity).div(dt);
            accelSource = "Derivative";
        } else {
            accelSource = "Hold";
        }
        if (rawAccel != null) {
            double tau = Constants.SOTMConstants.ACCEL_LPF_TIME_CONSTANT_SEC.get();
            double effectiveDt = dt > 1e-4 ? dt : Constants.UPDATE_LOOP_DT;
            double alpha = tau > 0.0 ? effectiveDt / (tau + effectiveDt) : 1.0;
            filteredFieldAcceleration =
                filteredFieldAcceleration.times(1.0 - alpha).plus(rawAccel.times(alpha));
        }
        previousFieldVelocity = currentFieldVelocity;
        previousFieldVelocityTimestampSec = now;
        Logger.recordOutput("SOTM/FieldAccelEstimate", filteredFieldAcceleration);
        Logger.recordOutput("SOTM/AccelSource", accelSource);

        Translation3d fieldAcceleration = new Translation3d(
            filteredFieldAcceleration.getX(), filteredFieldAcceleration.getY(), 0.0);

        movingShotSolution = ShooterStructure.Moving.solveMovingShot(
            target,
            robotVelocity,
            fieldAcceleration,
            getRotation(),
            currentChassisSpeeds.omegaRadiansPerSecond,
            Constants.SOTMConstants.MAX_ITERATIONS,
            Constants.SOTMConstants.TIME_TOLERANCE
        );
    }

    public ShooterStructure.MovingShotSolution getMovingShotSolution() {
        return movingShotSolution;
    }

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    public void periodic() {
        Logger.recordOutput("RobotState/PoseEstimation/PoseEstimation", getEstimatedPose());
        Logger.recordOutput("RobotState/PoseEstimation/Odometry", wheelOdometry.getPoseMeters());
        Logger.recordOutput("RobotState/PoseEstimation/FusedPose", fusedPose);
        Logger.recordOutput("RobotState/PoseEstimation/UsingFallback", !isFusedPoseFresh());

        // SOTM is now used unconditionally by the turret, so keep the solution fresh
        // every loop instead of only while a shoot trigger is held. Solver is cheap
        // (1-2 iterations typical, see SOTM/ConvergedIterations in logs).
        updateMovingShot();

//        if (DriverStation.isDisabled()) {
//            timer.stop();
//        }
//        if (autoStarted.update(DriverStation.isAutonomous())) {
//            shiftState = ShiftState.AUTO;
//            initialActiveHub = null;
//            activeHub = null;
//            timer.stop();
//            timer.reset();
//            Logger.recordOutput("RobotState/ShiftState", shiftState.name());
//        }
//        if (matchStarted.update(!Robot.isFirstRun())) {
//            timer.start();
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
//                    if (isShift()) {
//                        activeHub = (activeHub == Alliance.Blue) ? Alliance.Red : Alliance.Blue;
//                    }
//                    timer.restart();
//                    Logger.recordOutput("RobotState/ShiftState", shiftState.name());
//                    Logger.recordOutput("RobotState/ActiveHub", activeHub != null ? activeHub.name() : "BOTH");
//                }
//            }
//        }
//        if (currentAimState == AimState.MANUAL) {
//            return;
//        }
//        AimState desiredMode = calculateDesiredMode();
//        if (desiredMode != currentAimState) {
//            setAimState(desiredMode);
//        }
    }

    public boolean isAimedToScore() {
        return Subsystems.turret().atGoal() && !Subsystems.turret().isTraversing();
    }

    private AimState calculateDesiredMode() {
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty()) {
            return AimState.TO_HUB;
        }
        boolean inAllianceZone = isInAllianceZone(ourAlliance.get());
        if (inAllianceZone || areBothHubsActive()) {
            return AimState.TO_HUB;
        }
        if (isOurHubActive(ourAlliance.get())) {
            return AimState.TO_HUB;
        } else {
            return AimState.FERRY;
        }
    }

    public boolean isShift() {
        return shiftState == ShiftState.SHIFT1 ||
        shiftState == ShiftState.SHIFT2 ||
        shiftState == ShiftState.SHIFT3 ||
        shiftState == ShiftState.SHIFT4;
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

    @AutoLogOutput(key = "RobotState/TimeLeftInShift")
    public double timeLeftInShift() {
        return (shiftState.time - timer.get());
    }

    public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        gyroRotation = gyroAngle;
        rawGyroRotation = gyroAngle;
        wheelOdometry.resetPosition(gyroAngle, modulePositions, pose);

        // Seed the fused pose to the reset so getEstimatedPose() is correct
        // immediately, but mark it stale so we run on wheel odometry (also just
        // reset to `pose`) until the Pi confirms the reset with fresh output.
        fusedPose = pose;
        fusedSampleTimestamp = 0.0;
        fusedSeqnum = -1;
        fusedAppliedWallClock = Double.NEGATIVE_INFINITY;

        // Tell the Pi to reset its factor graph to this pose.
        resetRequestSeqnum++;
        resetRequestPose = pose;

        // clear and reinit buffers
        poseBuffer.clear();
        poseBuffer.addSample(Timer.getFPGATimestamp(), pose);
        // Stale wheel history would make the latency compensation measure motion
        // across the reset discontinuity, so it has to be re-seeded too.
        wheelPoseBuffer.clear();
        wheelPoseBuffer.addSample(Timer.getFPGATimestamp(), pose);
        intakeExtensionBuffer.clear();
        objectHistory.clear();
        Logger.recordOutput("RobotState/PoseReset", pose);
    }

    public void resetToPose(Pose2d newPose) {
        resetPose(
            newPose,
            gyroRotation,
            lastModulePositions
        );
    }

    public void addOdometryObservation(OdometryObservation observation) {
        if (observation.gyroAngle() != null) {
            gyroRotation = observation.gyroAngle();
            rawGyroRotation = observation.gyroAngle();
        }
        wheelOdometry.update(gyroRotation, observation.wheelPositions());
        latestOdometry = observation;
        // Buffer whatever the current best estimate is (fused when fresh, else
        // wheel-only) so RIO-side consumers (SOTM etc.) can query by timestamp.
        poseBuffer.addSample(observation.timestamp(), getEstimatedPose());
        wheelPoseBuffer.addSample(observation.timestamp(), wheelOdometry.getPoseMeters());
    }

    /**
     * Apply a fused pose received from the Orange Pi (via {@link
     * org.steelhawks.subsystems.poselink.PoseLink}). Rejects any observation
     * that is not strictly newer than the last applied one (by seqnum and
     * timestamp), so a reordered or duplicate UDP packet can never move the
     * pose backwards.
     *
     * @return true if the observation was applied.
     */
    public boolean applyFusedPose(FusedPoseObservation observation) {
        if (observation.seqnum() <= fusedSeqnum
            || observation.timestamp() < fusedSampleTimestamp) {
            Logger.recordOutput("RobotState/PoseLink/RejectedStale", true);
            return false;
        }
        fusedSeqnum = observation.seqnum();
        fusedSampleTimestamp = observation.timestamp();
        fusedPose = observation.pose();
        fusedAppliedWallClock = Timer.getFPGATimestamp();
        Logger.recordOutput("RobotState/PoseLink/RejectedStale", false);
        Logger.recordOutput("RobotState/PoseLink/AppliedPose", fusedPose);
        Logger.recordOutput("RobotState/PoseLink/QualityScore", observation.qualityScore());
        return true;
    }

    /** True while the last fused pose is within the staleness window. */
    public boolean isFusedPoseFresh() {
        return (Timer.getFPGATimestamp() - fusedAppliedWallClock) <= FUSED_STALENESS_TIMEOUT_SEC;
    }

    /** Latest odometry sample for PoseLink to forward to the Pi. */
    public Optional<OdometryObservation> getLatestOdometry() {
        return Optional.ofNullable(latestOdometry);
    }

    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return currentChassisSpeeds;
    }

    public long getResetRequestSeqnum() {
        return resetRequestSeqnum;
    }

    public Pose2d getResetRequestPose() {
        return resetRequestPose;
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

    /**
     * The robot's best field pose: the Pi's fused pose while it is fresh,
     * otherwise the locally-maintained wheel-only dead-reckoning pose. The
     * fallback runs continuously, so this recovers automatically the moment
     * fresh fused data resumes.
     *
     * <p>The fused pose is latency-compensated. What the Pi sends back is its
     * estimate at {@code fusedSampleTimestamp}, which is already 40-80 ms old by
     * the time it lands here (UDP out, a Pi cycle, the GTSAM solve, UDP back, a
     * RIO cycle). Using it raw leaves the pose standing that far behind the robot
     * - a fifth of a metre at 3 m/s. The old {@code SwerveDrivePoseEstimator} hid
     * this by replaying its odometry buffer forward after every vision
     * correction; splitting the estimator across the network means we have to do
     * that replay ourselves. So: take the wheel-odometry motion measured since the
     * Pi's sample and compose it onto the fused pose.
     */
    @AutoLogOutput(key = "RobotState/PoseEstimation/PoseEstimation")
    public Pose2d getEstimatedPose() {
        if (!isFusedPoseFresh()) {
            return wheelOdometry.getPoseMeters();
        }
        Optional<Pose2d> wheelAtFused = wheelPoseBuffer.getSample(fusedSampleTimestamp);
        if (wheelAtFused.isEmpty()) {
            // Sample aged out of the buffer (or none yet); the uncompensated pose
            // is still better than falling back to wheel-only.
            return fusedPose;
        }
        return fusedPose.plus(new Transform2d(wheelAtFused.get(), wheelOdometry.getPoseMeters()));
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
        double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle) {}

    /** A fused pose received from the Orange Pi over the vision link. */
    public record FusedPoseObservation(
        long seqnum,
        double timestamp,
        Pose2d pose,
        double qualityScore) {}

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
