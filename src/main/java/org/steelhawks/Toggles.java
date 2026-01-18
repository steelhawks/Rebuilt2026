package org.steelhawks;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.HashMap;
import java.util.Map;

public interface Toggles {

    LoggedNetworkBoolean debugMode =
        new LoggedNetworkBoolean("Toggles/DebugMode", true);
    LoggedNetworkBoolean tuningMode =
        new LoggedNetworkBoolean("Toggles/TuningMode", false);
    LoggedNetworkBoolean motionMagicEnabled =
        new LoggedNetworkBoolean("Toggles/MotionMagicEnabled", false);

    LoggedNetworkBoolean rateLimitSwerveEnabled =
        new LoggedNetworkBoolean("Toggles/RateLimitSwerveEnabled", false);

    class Vision {
        public static final LoggedNetworkBoolean visionEnabled =
            new LoggedNetworkBoolean("Toggles/Vision/VisionEnabled", true);
        public static final Map<String, LoggedNetworkBoolean> camerasEnabled;

        static {
            camerasEnabled = new HashMap<>();
            for (VisionConstants.CameraConfig config : VisionConstants.getCameraConfig()) {
                camerasEnabled.put(config.name(), new LoggedNetworkBoolean("Toggles/Vision/" + config.name() + "Enabled", true));
            }
        }
    }

    interface Swerve {
        LoggedNetworkBoolean driveOpenLoopOverride =
            new LoggedNetworkBoolean("Toggles/Swerve/DriveOpenLoopOverride", false);
        LoggedNetworkBoolean turnOpenLoopOverride =
            new LoggedNetworkBoolean("Toggles/Swerve/TurnOpenLoopOverride", false);
    }

    interface Flywheel {
        LoggedNetworkBoolean isEnabled =
            new LoggedNetworkBoolean("Toggles/Flywheel/IsEnabled", true);
        LoggedNetworkBoolean toggleVoltageOverride =
            new LoggedNetworkBoolean("Toggles/Flywheel/ToggleVoltageOverride", false);
        LoggedNetworkBoolean toggleCurrentOverride =
            new LoggedNetworkBoolean("Toggles/Flywheel/ToggleCurrentOverride", false);
    }
}
