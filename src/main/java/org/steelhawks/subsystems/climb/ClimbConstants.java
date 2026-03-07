package org.steelhawks.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.steelhawks.util.LoggedTunableNumber;

public class ClimbConstants {
    public static final int MOTOR_ID = 69;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/kI", 0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/kD", 0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Climb/kS", 0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Climb/kG", 0);
    // both of these are required for MotionMagic(R) expo
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Climb/kV", 0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Climb/kA", 0);

    public static final Rotation2d MIN_POSITION = new Rotation2d();
    public static final Rotation2d MAX_POSITION = new Rotation2d();

    public static final LoggedTunableNumber MOTIONMAGIC_EXPO_CRUISE_VELOCITY = new LoggedTunableNumber("Climb/CruiseVelocityRadPerSec", 0);

    public static final double REDUCTION = 23.0 / 1.0;

    public enum ClimbState {
        RETRACTED(0),
        L1(0),
        L2(0),
        L3(0);

        final double position;

        ClimbState(double position) {
            this.position = position;
        }
    }

    public static final double HOMING_VOLTS = 0;
    public static final double TOLERANCE = 0.1;
}
