package org.steelhawks.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class BatteryUtil {
    private static double lastMeasurement = -1;
    private static double loopTotalCurrent = 0;
    private static double ampHoursUsed = 0.0;
    private static double wattHoursUsed = 0.0;

    private BatteryUtil() {
        throw new InstantiationError("This is a utility class and cannot be instantiated.");
    }

    public static void reset() {
        loopTotalCurrent = 0;
    }

    public static void recordCurrentUsage(String device, double currentAmps) {
        loopTotalCurrent += currentAmps;
        Logger.recordOutput("BatteryUtil/Devices/" + device, currentAmps);
    }

    public static void integrateAndLogTotal() {
        double now = Timer.getFPGATimestamp();
        double voltage = RobotController.getBatteryVoltage();
        loopTotalCurrent += RobotController.getInputCurrent();

        Logger.recordOutput("BatteryUtil/TotalCurrentDraw", loopTotalCurrent);

        if (lastMeasurement < 0) {
            // first run, don't integrate yet
            lastMeasurement = now;
            return;
        }

        double dt = now - lastMeasurement;
        ampHoursUsed += (loopTotalCurrent * dt) / 3600.0;

        double power = voltage * loopTotalCurrent;
        wattHoursUsed += (power * dt) / 3600.0;

        lastMeasurement = now;
        Logger.recordOutput("BatteryUtil/AmpHoursUsed", ampHoursUsed);
        Logger.recordOutput("BatteryUtil/CurrentAmps", loopTotalCurrent);
        Logger.recordOutput("BatteryUtil/Power", power);
        Logger.recordOutput("BatteryUtil/WattHours", wattHoursUsed);
    }
}
