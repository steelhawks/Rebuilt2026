package org.steelhawks.pi.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.OptionalDouble;

/**
 * Converts Pi-local timestamps (what photonlib reports) into the RIO's clock,
 * which is the single timeline the graph and {@code FusedPoseOutput} use. Relies
 * on NT4's server-time sync; until it converges, {@link #toRioTime} returns
 * empty and callers should hold the observation out.
 */
public final class TimeSync {

    private TimeSync() {}

    /** Offset (seconds) to add to a local timestamp to get RIO/server time. */
    public static OptionalDouble offsetSeconds() {
        var micros = NetworkTableInstance.getDefault().getServerTimeOffset();
        return micros.isPresent()
            ? OptionalDouble.of(micros.getAsLong() * 1e-6)
            : OptionalDouble.empty();
    }

    public static OptionalDouble toRioTime(double localSeconds) {
        OptionalDouble off = offsetSeconds();
        return off.isPresent()
            ? OptionalDouble.of(localSeconds + off.getAsDouble())
            : OptionalDouble.empty();
    }
}
