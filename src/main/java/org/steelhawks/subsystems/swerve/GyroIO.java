package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d rollPosition = new Rotation2d();
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d yawPosition = new Rotation2d();
        public double accelerationXInGs = 0.0;
        public double accelerationYInGs = 0.0;
        // Gravity unit vector in sensor frame (g's). Combined with the raw
        // acceleration above this yields gravity-compensated linear acceleration.
        // Only the Pigeon path populates these; NavX/sim leave them at zero with
        // linearAccelerationAvailable=false.
        public double gravityVectorX = 0.0;
        public double gravityVectorY = 0.0;
        public double gravityVectorZ = 0.0;
        public boolean linearAccelerationAvailable = false;
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[]{};
        public Rotation2d[] odometryYawPositions = new Rotation2d[]{};
    }

    default void updateInputs(GyroIOInputs inputs) {}
}
