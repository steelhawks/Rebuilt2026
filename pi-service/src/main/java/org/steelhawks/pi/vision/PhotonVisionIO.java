package org.steelhawks.pi.vision;

import java.util.List;

/** Source of decoded PhotonVision observations. Swappable for replay/sim. */
public interface PhotonVisionIO {
    /** All observations decoded since the last call, timestamped in RIO time. */
    List<CameraObservation> poll();

    /** True if every camera is connected. */
    default boolean allConnected() {
        return true;
    }
}
