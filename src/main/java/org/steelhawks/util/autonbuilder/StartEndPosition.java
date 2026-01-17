package org.steelhawks.util.autonbuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum StartEndPosition {
    NOTHING_AUTO(3.0, 3.0, 0.0),
    ;
    public final double x;
    public final double y;
    public final double rotRadians;

    StartEndPosition(double x, double y, double rotRadians) {
        this.x = x;
        this.y = y;
        this.rotRadians = rotRadians;
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(rotRadians));
    }
}
