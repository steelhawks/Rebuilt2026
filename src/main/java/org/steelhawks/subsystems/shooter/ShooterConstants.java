package org.steelhawks.subsystems.shooter;

import org.steelhawks.util.LoggedTunableNumber;

public class ShooterConstants {
    public static final int SHOOTER_MOTOR_1_ID = 0;
    public static final int SHOOTER_MOTOR_2_ID = 0;
    public static final double SHOOT_SPEED = 0;
    public static final double GEAR_RATIO = 2;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0);

    public static final double TOLERANCE = 0.0;

    public static final int NUMBER_OF_SAMPLES = 67;
}
