package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {

    public static final double REDUCTION = 1.0 / 1.0;

    public enum State {
		// TODO In radians, need to tune
		RETRACTED(new Rotation2d(0.0)),
		HOME(new Rotation2d(0.0)),
		INTAKE(new Rotation2d(0.0));

		private final Rotation2d positionRad;

		State(Rotation2d positionRad) {
			this.positionRad = positionRad;
		}

		public Rotation2d getPosition() {
			return positionRad;
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

    // TODO SET IDS
    public static final int LEFT_MOTOR_ID = 13;
    public static final int RIGHT_MOTOR_ID = 14;
    public static final int INTAKE_MOTOR_ID = 15;
    public static final int ENCODER_ID = 16;

	public static final Rotation2d MAX_ROTATION = new Rotation2d();
	public static final Rotation2d MIN_ROTATION = new Rotation2d();

    public static final Rotation2d MAG_OFFSET = Rotation2d.fromRotations(0.0);
}
