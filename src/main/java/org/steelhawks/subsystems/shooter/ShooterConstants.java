package org.steelhawks.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.TunableNumber;

public class ShooterConstants {

	public static final int MOTOR_ID = 0;

	public static final double TOLERANCE = 5.0;
	public static final double GEAR_REDUCTION = 1.0;
	public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(2.0);

	public static final double MAX_RPM = 0.0;
	public static final double MAX_VELOCITY_ROT_PER_SEC = MAX_RPM / 60;
	public static final double MAX_VELOCITY_METERS_PER_SEC = MAX_VELOCITY_ROT_PER_SEC * 2 * Math.PI * FLYWHEEL_RADIUS;

	public static final double MAX_ACCELERATION_ROT_PER_SEC_2 = 0.0;

	public static final LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/kP", Constants.omega(0.0, 0.0));
	public static final double KI = 0.0;
	public static final double KD = 0.0;

	public static final double kS = 0.0;
	public static final double kV = 0.0;

}
