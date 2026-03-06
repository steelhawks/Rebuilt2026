package org.steelhawks.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

	@AutoLog
	class SpindexerIOInputs {
		public boolean connected = false;
		public double positionRad = 0.0;
		public double velocityRadPerSec = 0.0;
		public double appliedVolts = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double torqueCurrentAmps = 0.0;
		public double tempCelsius = 0.0;
	}

    @AutoLog
    class FeederIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
    }

	default void updateInputs(SpindexerIOInputs spindexerInputs, FeederIOInputs feederInputs) {}

	default void setBrakeMode(boolean enabled) {}

	default void runSpindexer(double output) {}

    default void runFeeder(double output) {}

	default void stopSpindexer() {}

    default void stopFeeder() {}
}
