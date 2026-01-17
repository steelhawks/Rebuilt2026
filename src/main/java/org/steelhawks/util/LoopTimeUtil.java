package org.steelhawks.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class LoopTimeUtil {

    private static double startTime = -1;

    private LoopTimeUtil() {
        throw new InstantiationError("This is a utility class and cannot be instantiated.");
    }

    public static void reset() {
        startTime = Timer.getFPGATimestamp();
    }

    public static void record(String subsystem) {
        double now = Timer.getFPGATimestamp();
        Logger.recordOutput("LoopTimes/" + subsystem + "ms", (now - startTime) * 1000.0);
        startTime = now;
    }
}
