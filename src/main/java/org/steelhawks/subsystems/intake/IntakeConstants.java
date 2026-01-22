package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {

	public enum State {
		// In radians, need to tune
		RETRACTED(0.0),
		HOME(0.0),
		INTAKE(0.0);

		private final double positionRad;

		State(double positionRad) {
			this.positionRad = positionRad;
		}

		public Rotation2d getPosition() {
			return new Rotation2d(positionRad);
		}
	}

	public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.0);
	public static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
	public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);
	public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.0);
	public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG", 0.0);
	public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA", 0.0);
	public static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.0);

	public static final LoggedTunableNumber MAX_VELOCITY_RAD_PER_SEC = new LoggedTunableNumber("Intake/MaxVelocityRadPerSec", 0.0);
	public static final LoggedTunableNumber MAX_ACCEL_RAD_PER_SEC_SQ = new LoggedTunableNumber("Intake/MaxAccelRadPerSecSQ", 0.0);
	public static final double TOLERANCE = 0.02;

	public static final Rotation2d MAX_ROTATION = new Rotation2d();
	public static final Rotation2d MIN_ROTATION = new Rotation2d();
}
