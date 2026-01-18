package org.steelhawks.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

	@AutoLog
	class ShooterIOInputs {
		public boolean shouldRunProfile = false;
		public double goal = 0;

		public boolean connected = false;
		public double positionRad = 0;
		public double velocityRadPerSec = 0;
		public double appliedVolts = 0;
		public double currentAmps = 0;
		public double tempCelsius = 0;
	}

	default void updateInputs(ShooterIOInputs inputs) {}

	default void runOpenLoop(double output) {}

	default void runSpeed(double speed) {}

	default void zeroEncoders() {}

	default void setPID(double kP, double kI, double kD) {}

	default void setBrakeMode(boolean enabled) {}

	default void stop() {}

}
