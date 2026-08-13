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
    private final StatusSignal<Double> gravityVectorX;
    private final StatusSignal<Double> gravityVectorY;
    private final StatusSignal<Double> gravityVectorZ;
    private final DoubleRingBuffer yawPositionQueue;
    private final DoubleRingBuffer yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity;

    // Mount pose as calibrated on the device, kept only for logging so an
    // uncalibrated Pigeon is visible in the log instead of quietly biasing SOTM.
    private final double mountPoseYawDeg;
    private final double mountPosePitchDeg;
    private final double mountPoseRollDeg;

    public GyroIOPigeon2(int pigeon2Id, CANBus canBus) {
        pigeon = new Pigeon2(pigeon2Id, canBus);

        roll = pigeon.getRoll();
        pitch = pigeon.getPitch();
        yaw = pigeon.getYaw();
        accelerationX = pigeon.getAccelerationX();
        accelerationY = pigeon.getAccelerationY();
        gravityVectorX = pigeon.getGravityVectorX();
        gravityVectorY = pigeon.getGravityVectorY();
        gravityVectorZ = pigeon.getGravityVectorZ();
        yawVelocity = pigeon.getAngularVelocityZDevice();

        // Mount pose lives on the device (Tuner X's "Mount Calibration" writes it
        // there) and the firmware needs it to report a correct gravity vector and
        // tilt-aware yaw. We deliberately do NOT apply a default-constructed
        // Pigeon2Configuration here: that call zeroes MountPose.{Yaw,Pitch,Roll} on
        // every boot, silently undoing the calibration. The fallout lands on SOTM,
        // which subtracts the gravity vector from the raw accelerometer to get linear
        // acceleration (see Swerve.periodic):
        //   - residual tilt leaves a standing bias of g*sin(theta), so 2 deg of mount
        //     error is 0.34 m/s^2 of phantom acceleration even while parked;
        //   - a mount YAW error rotates the acceleration vector out of the robot
        //     frame, so the shot lead is applied in the wrong direction entirely.
        // Nothing else in this class depends on a factory-default config, so read the
        // device's calibration back for logging and otherwise leave it alone.
        Pigeon2Configuration onDeviceConfig = new Pigeon2Configuration();
        PhoenixUtil.tryUntilOk(5, () -> pigeon.getConfigurator().refresh(onDeviceConfig));
        mountPoseYawDeg = onDeviceConfig.MountPose.MountPoseYaw;
        mountPosePitchDeg = onDeviceConfig.MountPose.MountPosePitch;
        mountPoseRollDeg = onDeviceConfig.MountPose.MountPoseRoll;
        pigeon.getConfigurator().setYaw(0.0);
        roll.setUpdateFrequency(50.0);
        pitch.setUpdateFrequency(50.0);
        yaw.setUpdateFrequency(Swerve.ODOMETRY_FREQUENCY);

        yawVelocity.setUpdateFrequency(50.0);
        accelerationX.setUpdateFrequency(100.0);
        accelerationY.setUpdateFrequency(100.0);
        gravityVectorX.setUpdateFrequency(100.0);
        gravityVectorY.setUpdateFrequency(100.0);
        gravityVectorZ.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

        PhoenixUtil.registerSignals(
            canBus,
            roll, pitch, yaw,
            accelerationX, accelerationY,
            gravityVectorX, gravityVectorY, gravityVectorZ,
            yawVelocity);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(yaw, accelerationX, accelerationY, yawVelocity);
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.accelerationXInGs = accelerationX.getValueAsDouble();
        inputs.accelerationYInGs = accelerationY.getValueAsDouble();
        inputs.gravityVectorX = gravityVectorX.getValueAsDouble();
        inputs.gravityVectorY = gravityVectorY.getValueAsDouble();
        inputs.gravityVectorZ = gravityVectorZ.getValueAsDouble();
        inputs.linearAccelerationAvailable =
            BaseStatusSignal.isAllGood(accelerationX, accelerationY, gravityVectorX, gravityVectorY);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        Logger.recordOutput("Swerve/Gyro/AccelerationInGs", Math.hypot(inputs.accelerationXInGs, inputs.accelerationYInGs));
        // Diagnostics for the SOTM acceleration path. With the robot flat and still:
        //   GravityHorizontalComponent should be ~0 - anything above ~0.035 (2 deg of
        //     tilt) means the mount pose is not calibrated and is feeding SOTM a
        //     standing acceleration bias;
        //   LinearAccelMagnitudeMps2 is the raw noise floor - read it off a log with
        //     the robot driving to size SOTM/AccelDeadbandMps2 and SOTM/AccelMaxMps2.
        Logger.recordOutput(
            "Swerve/Gyro/GravityHorizontalComponent",
            Math.hypot(inputs.gravityVectorX, inputs.gravityVectorY));
        Logger.recordOutput(
            "Swerve/Gyro/LinearAccelMagnitudeMps2",
            Math.hypot(
                inputs.accelerationXInGs - inputs.gravityVectorX,
                inputs.accelerationYInGs - inputs.gravityVectorY) * 9.80665);
        Logger.recordOutput("Swerve/Gyro/MountPose/YawDeg", mountPoseYawDeg);
        Logger.recordOutput("Swerve/Gyro/MountPose/PitchDeg", mountPosePitchDeg);
        Logger.recordOutput("Swerve/Gyro/MountPose/RollDeg", mountPoseRollDeg);

        int sampleCount = yawTimestampQueue.size();
        inputs.odometryYawTimestamps = new double[sampleCount];
        inputs.odometryYawPositions = new Rotation2d[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            inputs.odometryYawTimestamps[i] = yawTimestampQueue.poll();
            inputs.odometryYawPositions[i] = Rotation2d.fromDegrees(yawPositionQueue.poll());
        }
    }
}
