package org.steelhawks.pi.vision;

import java.util.OptionalDouble;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Frame timestamps, on the clock the factor graph runs on.
 *
 * <p>There is nothing to convert. PhotonVision runs its own time-sync protocol
 * (ping/pong, independent of NetworkTables) and applies the offset itself, so
 * {@code PhotonPipelineMetadata.captureTimestampMicros} is already documented as
 * being "in nt::Now on the time sync server".
 *
 * <p>This class used to add {@code NetworkTableInstance.getServerTimeOffset()} on
 * top of that, which double-corrected. In the 2026-06-06 basement log it put
 * vision timestamps ~1.78e9 s away from the odometry clock, and later wandered
 * over a ~4000 s range as the NT offset moved. Every observation then failed the
 * {@code nodes_[i].t < v.t} test in {@code PoseGraph::placeVision}, exited the
 * scan at {@code i == 0}, and got pinned to the oldest node in the buffer instead
 * of its real capture time. Parked that is invisible - every node holds the same
 * pose - which is why the Pi logs looked calm while the robot behaved badly.
 *
 * <p>What still needs doing is refusing frames whose sync has not converged.
 * {@code timeSinceLastPong} is {@code Long.MAX_VALUE} until the first pong lands
 * and grows whenever the sync link drops; a stale pong means the offset already
 * baked into the capture timestamp is stale too.
 */
public final class TimeSync {

    private TimeSync() {}

    /** Drop frames whose last time-sync pong is older than this (microseconds). */
    public static final long MAX_PONG_AGE_MICROS = 5_000_000L;

    /**
     * The frame's capture time in RIO/server seconds, or empty if PhotonVision's
     * time sync has not converged (or has gone stale) and the timestamp cannot be
     * trusted.
     */
    public static OptionalDouble captureTime(PhotonPipelineResult result) {
        long pongAge = result.metadata.timeSinceLastPong;
        if (pongAge < 0 || pongAge > MAX_PONG_AGE_MICROS) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(result.getTimestampSeconds());
    }

    /** Age of the last time-sync pong, in milliseconds; for logging. */
    public static double pongAgeMillis(PhotonPipelineResult result) {
        return result.metadata.timeSinceLastPong / 1000.0;
    }
}
