package org.steelhawks.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

	@AutoLog
	class SpindexerIOInputs {
		public boolean connected = false;
		public double positionRad = 0.0;
		public double velocityRadPerSec = 0.0;
		public double appliedVolts = 0.0;
		public double currentAmps = 0.0;
		public double torqueCurrentAmps = 0.0;
		public double tempCelsius = 0.0;
	}

	default void updateInputs(SpindexerIOInputs inputs) {}

	default void setBrakeMode(boolean enabled) {}

	default void runSpindexer(double output) {}

	default void stop() {}
}
