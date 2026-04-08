package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotConfig.CANBusList;
import org.steelhawks.RobotContainer;
import org.steelhawks.util.PhoenixUtil;


public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final DoubleRingBuffer yawPositionQueue;
    private final DoubleRingBuffer yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity;

    public GyroIOPigeon2(int pigeon2Id, CANBus canBus) {
        pigeon = new Pigeon2(pigeon2Id, canBus);

        roll = pigeon.getRoll();
        pitch = pigeon.getPitch();
        yaw = pigeon.getYaw();
        accelerationX = pigeon.getAccelerationX();
        accelerationY = pigeon.getAccelerationY();
        yawVelocity = pigeon.getAngularVelocityZDevice();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        roll.setUpdateFrequency(50.0);
        pitch.setUpdateFrequency(50.0);
        yaw.setUpdateFrequency(Swerve.ODOMETRY_FREQUENCY);

        yawVelocity.setUpdateFrequency(50.0);
        accelerationX.setUpdateFrequency(50.0);
        accelerationY.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

        PhoenixUtil.registerSignals(
            canBus,
            roll, pitch, yaw, accelerationX, accelerationY, yawVelocity);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(yaw, accelerationX, accelerationY, yawVelocity);
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.accelerationXInGs = accelerationX.getValueAsDouble();
        inputs.accelerationYInGs = accelerationY.getValueAsDouble();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        Logger.recordOutput("Swerve/Gyro/AccelerationInGs", Math.hypot(inputs.accelerationXInGs, inputs.accelerationYInGs));

        int sampleCount = yawTimestampQueue.size();
        inputs.odometryYawTimestamps = new double[sampleCount];
        inputs.odometryYawPositions = new Rotation2d[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            inputs.odometryYawTimestamps[i] = yawTimestampQueue.poll();
            inputs.odometryYawPositions[i] = Rotation2d.fromDegrees(yawPositionQueue.poll());
        }
    }
}
