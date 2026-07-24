package org.steelhawks.pi.vision;

import edu.wpi.first.math.geometry.Pose3d;

/** One decoded PhotonVision robot-pose observation, timestamped in RIO time. */
public record CameraObservation(
    int cameraIndex,
    double timestamp,
    Pose3d robotPose,
    double ambiguity,
    int tagCount,
    double avgTagDistance,
    int[] tagIds) {}
