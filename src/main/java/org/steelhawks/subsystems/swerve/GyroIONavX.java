package org.steelhawks.subsystems.swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;


public class GyroIONavX implements GyroIO {

    private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) Swerve.ODOMETRY_FREQUENCY);
    private final DoubleRingBuffer yawPositionQueue;
    private final DoubleRingBuffer yawTimestampQueue;

    public GyroIONavX() {
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

        int sampleCount = yawTimestampQueue.size();
        inputs.odometryYawTimestamps = new double[sampleCount];
        inputs.odometryYawPositions = new Rotation2d[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            inputs.odometryYawTimestamps[i] = yawTimestampQueue.poll();
            inputs.odometryYawPositions[i] = Rotation2d.fromDegrees(-yawPositionQueue.poll());
        }
    }
}
