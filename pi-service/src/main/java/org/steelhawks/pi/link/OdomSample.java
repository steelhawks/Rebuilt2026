package org.steelhawks.pi.link;

import edu.wpi.first.math.geometry.Pose2d;
import org.steelhawks.proto.AllianceColor;

/** One decoded {@code RobotOdomInputs} packet from the RIO. */
public record OdomSample(
    long seqnum,
    double timestamp,
    Pose2d odomPose,
    boolean isOnBump,
    AllianceColor alliance,
    long configHash,
    long resetSeqnum,
    Pose2d resetPose,
    long sessionId) {}
