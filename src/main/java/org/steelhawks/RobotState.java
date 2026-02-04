package org.steelhawks;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.subsystems.swerve.Swerve;

import java.util.*;

public class RobotState {

    private static final double poseBufferSizeSec = 2.0;
    private static final double turretAngleBufferSizeSec = 2.0;
    private static final double objectMaxAgeSec = 1.0;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
        TimeInterpolatableBuffer.createBuffer(turretAngleBufferSizeSec);
    private final List<TimestampedObjectList> objectHistory;
    private List<DetectedObject> currentDetectedObjects;


    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(Objects.requireNonNull(Swerve.getModuleTranslations()));
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    private Rotation2d gyroRotation = new Rotation2d();
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final SwerveDriveOdometry wheelOdometry =
        new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private final List<DetectedObject> detectedObjects;
    private final List<VisionObservation> visionObservations;

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private RobotState() {
        this.detectedObjects = new ArrayList<>();
        this.visionObservations = new ArrayList<>();

        this.objectHistory = new ArrayList<>();
        this.currentDetectedObjects = new ArrayList<>();
    }

    public boolean isHubActive() {
        String gameData = DriverStation.getGameSpecificMessage();
        return !gameData.isEmpty()
            && gameData.charAt(0) == 'B'
            && DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);
    }

    public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        gyroRotation = gyroAngle;
        rawGyroRotation = gyroAngle;
        poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
        wheelOdometry.resetPosition(gyroAngle, modulePositions, pose);

        // clear and reinit buffers
        poseBuffer.clear();
        poseBuffer.addSample(Timer.getFPGATimestamp(), pose);
        turretAngleBuffer.clear();
        objectHistory.clear();
        visionObservations.clear();
        detectedObjects.clear();
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
        visionObservations.add(observation);
        Logger.recordOutput("RobotState/VisionObservations",
            visionObservations.toArray(new VisionObservation[0]));
        Logger.recordOutput("RobotState/LatestVisionPose", observation.robotPose());
    }

    public void addObjectDetections(List<DetectedObject> objects, double timestamp) {
        currentDetectedObjects = new ArrayList<>(objects);

        // Add to history
        objectHistory.add(new TimestampedObjectList(timestamp, new ArrayList<>(objects)));

        // Remove old history entries
        double now = Timer.getFPGATimestamp();
        objectHistory.removeIf(entry -> now - entry.timestamp > objectMaxAgeSec);

        // Log detected objects
        Pose3d[] objectPoses = objects.stream()
            .map(DetectedObject::pose)
            .toArray(Pose3d[]::new);
        Logger.recordOutput("RobotState/DetectedObjects", objectPoses);
        Logger.recordOutput("RobotState/DetectedObjectCount", objects.size());
    }

    public void addTurretAngle(Rotation2d angle, double timestamp) {
        turretAngleBuffer.addSample(timestamp, angle);
    }


    @AutoLogOutput(key = "RobotState/PoseEstimation/PoseEstimation")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
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
    @AutoLogOutput(key = "RobotState/Turret/AngleAtTime")
    public Optional<Rotation2d> getTurretAngleAtTime(double timestamp) {
        return turretAngleBuffer.getSample(timestamp);
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
        if (currentDetectedObjects.isEmpty()) {
            return Optional.empty();
        }

        Pose2d robotPose = getEstimatedPose();
        return currentDetectedObjects.stream()
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
