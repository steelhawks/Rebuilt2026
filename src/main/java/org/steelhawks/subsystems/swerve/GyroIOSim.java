package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.util.Units;
import org.steelhawks.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation mGyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.mGyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = mGyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
            mGyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = mGyroSimulation.getCachedGyroReadings();
    }
}
