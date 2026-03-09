package org.steelhawks.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

	@AutoLog
	class SpindexerIOInputs {
		public boolean motor1Connected = false;
		public double motor1PositionRad = 0.0;
		public double motor1VelocityRadPerSec = 0.0;
		public double motor1AppliedVolts = 0.0;
		public double motor1CurrentAmps = 0.0;
		public double motor1TorqueCurrentAmps = 0.0;
		public double motor1TempCelsius = 0.0;

		public boolean motor2Connected = false;
		public double motor2PositionRad = 0.0;
		public double motor2VelocityRadPerSec = 0.0;
		public double motor2AppliedVolts = 0.0;
		public double motor2CurrentAmps = 0.0;
		public double motor2TorqueCurrentAmps = 0.0;
		public double motor2TempCelsius = 0.0;
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
