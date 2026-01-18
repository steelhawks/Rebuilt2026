package org.steelhawks.subsystems.shooter;


import com.ctre.phoenix6.CANBus;
import org.steelhawks.util.LoggedTunableNumber;

public class ShooterConstants {

    // ALL VALUES ARE PLACEHOLDER VALUES AND NEED TO BE CHANGED

    public static final CANBus CLAW_CANBUS =  new CANBus("");
    public static final int CAN_RANGE_ID = 17;
    public static final int GEAR_RATIO = 2 / 1;

    public static final Integer SHOOTER_MOTOR_ID = 1;
    public static final Double SHOOTER_GEAR_RATIO = 2.0;
    public static final Integer SHOOTER_MOTOR_ID2 = 2;

    public static final Double SHOOTER_SPEED = 0.15;
    public static final Double SHOOTER_SLOW_SPEED = 0.125;
    public static final Double SHOOTER_MOTOR_MAX_RPM = 6784.0;

    public static final LoggedTunableNumber SHOOTER_KP = new LoggedTunableNumber("SHOOTER_KP", 2.4);
    public static final LoggedTunableNumber SHOOTER_KI = new LoggedTunableNumber("SHOOTER_KI", 0);
    public static final LoggedTunableNumber SHOOTER_KD = new LoggedTunableNumber("SHOOTER_KP", 0.1);
    public static final LoggedTunableNumber SHOOTER_KS = new LoggedTunableNumber("SHOOTER_KS", 0.2);
    public static final LoggedTunableNumber SHOOTER_KV = new LoggedTunableNumber("SHOOTER_KV", 0.16);



}
