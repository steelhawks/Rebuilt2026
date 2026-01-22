package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

	@AutoLog
	class IntakeIOInputs {
		public double goal = 0;

		public boolean connected = false;
		public Rotation2d positionRad = new Rotation2d();
		public Rotation2d velocityRadPerSec = new Rotation2d();
		public double appliedVolts = 0.0;
		public double currentAmps = 0.0;
		public double torqueCurrentAmps = 0.0;
		public double tempCelsius = 0.0;
	}

	default void updateInputs(IntakeIOInputs inputs) {}

	default void setBrakeMode(boolean enabled) {}

	default void runPivotPosition(double position, double feedforward) {}

	default void runPivotOpenLoop(double output, boolean isTorqueCurrent) {}

	default void setPivotPID(double kP, double kI, double kD) {}

	default void stop() {}
}
