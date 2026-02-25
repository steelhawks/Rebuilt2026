package org.steelhawks;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.steelhawks.subsystems.vision.VisionConstants;

import java.util.HashMap;
import java.util.Map;

public interface Toggles {

    LoggedNetworkBoolean debugMode =
        new LoggedNetworkBoolean("Toggles/DebugMode", false);
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
        LoggedNetworkBoolean toggleAdaptiveFeedforward =
            new LoggedNetworkBoolean("Toggles/Flywheel/ToggleAdaptiveFeedforward", true);
    }

    interface Turret {
        LoggedNetworkBoolean isEnabled =
            new LoggedNetworkBoolean("Toggles/Turret/IsEnabled", true);
        LoggedNetworkBoolean toggleVoltageOverride =
            new LoggedNetworkBoolean("Toggles/Turret/ToggleVoltageOverride", false);
        LoggedNetworkBoolean toggleCurrentOverride =
            new LoggedNetworkBoolean("Toggles/Turret/ToggleCurrentOverride", false);
    }

    interface Intake {
        LoggedNetworkBoolean isEnabled =
            new LoggedNetworkBoolean("Toggles/Intake/IsEnabled", true);
        LoggedNetworkBoolean toggleVoltageOverride =
            new LoggedNetworkBoolean("Toggles/Intake/ToggleVoltageOverride", false);
        LoggedNetworkBoolean toggleCurrentOverride =
            new LoggedNetworkBoolean("Toggles/Intake/ToggleCurrentOverride", false);
    }

    interface Indexer {
        LoggedNetworkBoolean isSpindexerEnabled =
            new LoggedNetworkBoolean("Toggles/Indexer/Spindexer/IsEnabled", true);
        LoggedNetworkBoolean isFeederEnabled =
            new LoggedNetworkBoolean("Toggles/Indexer/Feeder/IsEnabled", true);
    }

    interface Hood {
        LoggedNetworkBoolean isEnabled =
            new LoggedNetworkBoolean("Toggles/Hood/IsEnabled", true);
        LoggedNetworkBoolean disableBrakeMode =
            new LoggedNetworkBoolean("Toggles/Hood/DisableBrakeMode", true);
        LoggedNetworkBoolean currentOverride =
            new LoggedNetworkBoolean("Toggles/Hood/ToggleCurrentOverride");
        LoggedNetworkBoolean voltageOverride =
            new LoggedNetworkBoolean("Toggles/Hood/ToggleVoltageOverride");
    }
}
