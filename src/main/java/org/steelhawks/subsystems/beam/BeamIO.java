package org.steelhawks.subsystems.beam;

import org.littletonrobotics.junction.AutoLog;

public interface BeamIO {

    @AutoLog
    class BeamIOInputs {
        public boolean connected = false;
        public boolean detected = false;
        public double distanceMeters = 0.0;
    }

    default void updateInputs(BeamIOInputs inputs) {}

    record BeamBreakConfig(int id, double proximityHysteresis, double proximityThreshold, UpdateMode mode, double frequency) {}

    enum UpdateMode {
        SHORT_RANGE,
        LONG_RANGE
    }
}
