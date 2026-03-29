package org.steelhawks;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.steelhawks.subsystems.swerve.Swerve;

import java.util.Objects;

public class RobotState {

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(Objects.requireNonNull(Swerve.getModuleTranslations()));
    private final Rotation2d gyroAngle = new Rotation2d();
    private final Pose2d initialPoseMeters = new Pose2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };


    private static RobotState instance;
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, lastModulePositions, initialPoseMeters);

    private RobotState() {
        // Stuff will go in here later
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
}
