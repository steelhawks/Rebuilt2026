package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double accelerationXInGs = 0.0;
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[]{};
        public Rotation2d[] odometryYawPositions = new Rotation2d[]{};
    }

    default void updateInputs(GyroIOInputs inputs) {}
}
