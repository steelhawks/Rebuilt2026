package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

	@AutoLog
	class IntakeIOInputs {
		public double goal = 0;

		public boolean leftConnected = false;
		public Rotation2d leftPositionRad = new Rotation2d();
		public double leftVelocityRadPerSec = 0.0;
		public double leftAppliedVolts = 0.0;
		public double leftCurrentAmps = 0.0;
		public double leftTorqueCurrentAmps = 0.0;
		public double leftTempCelsius = 0.0;

        public boolean rightConnected = false;
        public Rotation2d rightPositionRad = new Rotation2d();
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTorqueCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;

        public boolean intakeConnected = false;
        public Rotation2d intakePositionRad = new Rotation2d();
        public double intakeVelocityRadPerSec = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrentAmps = 0.0;
        public double intakeTorqueCurrentAmps = 0.0;
        public double intakeTempCelsius = 0.0;

        public boolean encConnected = false;
        public Rotation2d encAbsPositionRad = new Rotation2d();
        public double encVelocityRadPerSec = 0.0;
        public double encAppliedVolts = 0.0;
	}

	default void updateInputs(IntakeIOInputs inputs) {}

	default void setBrakeMode(boolean enabled) {}

	default void runPivotPosition(double position, double feedforward) {}

	default void runPivotOpenLoop(double output, boolean isTorqueCurrent) {}

    default void runPivotPercentOut(double output) {}

	default void setPivotPID(double kP, double kI, double kD) {}

    default void runIntake(double output) {}

	default void stopPivot() {}

    default void stopIntake() {}
}
