package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.util.LoggedTunableNumber;

public class IntakeConstants {

    public static final double REDUCTION = (7.0 / 1.0);
    public static final double PINION_RADIUS = Units.inchesToMeters(1.033922 / 2.0);
    public static final double METERS_PER_ROTATION = (2 * Math.PI * PINION_RADIUS);
    public static final double METERS_PER_RADIAN = PINION_RADIUS / REDUCTION;
    public static final double MASS_KG = 0.0;

    public static final double INTAKE_SPEED = 1.0;
    public static final Rotation2d RACK_ANGLE = Rotation2d.fromDegrees(-19.0);

    public enum State {
        // TODO In meters, need to tune
        RETRACTED(0.15),
        CENTER_OF_MOTION(0.15),
        HOME(MAX_EXTENSION_FROM_FRAME - 0.33),
        INTAKE(0.3);

        private final double positionMeters;

        State(double positionMeters) {
            this.positionMeters = positionMeters;
        }

        public double getPosition() {
            return positionMeters;
        }
    }

    public static final double MAX_EXTENSION_FROM_FRAME = Units.inchesToMeters(12.0 + (1.0 / 8.0));

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 5.0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA", 0.0);

    public static final LoggedTunableNumber MAX_VELOCITY_RAD_PER_SEC = new LoggedTunableNumber("Intake/MaxVelocityMetersPerSec", 0.05);
    public static final LoggedTunableNumber MAX_ACCEL_RAD_PER_SEC_SQ = new LoggedTunableNumber("Intake/MaxAccelRadMetersSecSq", 0.08);

    public static final double TOLERANCE = 0.02;

//    0.144461m
    public static final double MIN_EXTENSION = MAX_EXTENSION_FROM_FRAME - 0.33 - 0.02;

    public static final int LEFT_ID = 1;
    public static final int RIGHT_ID = 2;
    public static final int INTAKE_ID = 3;
}
