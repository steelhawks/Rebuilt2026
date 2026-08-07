package org.steelhawks.pi;

import java.io.InputStream;
import java.util.Properties;
import org.littletonrobotics.junction.Logger;

/**
 * Build provenance stamped into the jar by the {@code poselinkBuildInfo} Gradle
 * task. Without this there is no way to tell which build produced a given wpilog,
 * which makes "is this log from before or after the fix?" unanswerable - and on a
 * service deployed over SSH, whether the Pi is even running what you think it is.
 */
public final class BuildInfo {

    private BuildInfo() {}

    private static final String RESOURCE = "/poselink-build.properties";

    private static final Properties PROPS = load();

    private static Properties load() {
        Properties props = new Properties();
        try (InputStream in = BuildInfo.class.getResourceAsStream(RESOURCE)) {
            if (in != null) {
                props.load(in);
            }
        } catch (Exception e) {
            // Provenance is diagnostic only; never let it stop the service starting.
        }
        return props;
    }

    /** Git SHA of this build, or {@code "unknown"}. Sent to the RIO over the link. */
    public static String gitSha() {
        return PROPS.getProperty("gitSha", "unknown");
    }

    /** Record the build stamp as log metadata. Must run before {@code Logger.start()}. */
    public static void record() {
        Logger.recordMetadata("GitSHA", gitSha());
        Logger.recordMetadata("GitBranch", PROPS.getProperty("gitBranch", "unknown"));
        Logger.recordMetadata("GitDirty", PROPS.getProperty("gitDirty", "unknown"));
        Logger.recordMetadata("BuildDate", PROPS.getProperty("buildDate", "unknown"));
    }
}
