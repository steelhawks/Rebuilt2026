package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {

    public static final double REDUCTION = (4.0 / 1.0);
    public static final double PINION_RADIUS = Units.inchesToMeters(1.033922 / 2.0);
    public static final double METERS_PER_RADIAN = (2 * Math.PI * PINION_RADIUS) / REDUCTION;
    public static final double MASS_KG = 0.0;

    public static final double INTAKE_SPEED = 1.0;
    public static final Rotation2d RACK_ANGLE = Rotation2d.fromDegrees(35.0);

    public enum State {
        // TODO In meters, need to tune
        RETRACTED(0.0),
        HOME(0.0),
        INTAKE(0.80);

        private final double positionMeters;

        State(double positionMeters) {
            this.positionMeters = positionMeters;
        }

        public double getPosition() {
            return positionMeters;
        }
    }

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", Constants.omega(0, 900.0));
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 5.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 1);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA", 0.0);

    public static final LoggedTunableNumber MAX_VELOCITY_RAD_PER_SEC = new LoggedTunableNumber("Intake/MaxVelocityMetersPerSec", Constants.omega(0, 2));
    public static final LoggedTunableNumber MAX_ACCEL_RAD_PER_SEC_SQ = new LoggedTunableNumber("Intake/MaxAccelMetersSecSq", Constants.omega(0, 5));
    public static final double TOLERANCE = 0.02;

    public static final double MAX_EXTENSION = Units.inchesToMeters(17.706);
    public static final double MIN_EXTENSION = 0.0;

    public static final int LEFT_ID = 60;
    public static final int RIGHT_ID = 61;
    public static final int INTAKE_ID = 62;
}
