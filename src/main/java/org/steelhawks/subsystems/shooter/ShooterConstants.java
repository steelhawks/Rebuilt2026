package org.steelhawks.subsystems.shooter;


import com.ctre.phoenix6.CANBus;

public class ShooterConstants {

    // ALL VALUES ARE PLACEHOLDER VALUES AND NEED TO BE CHANGED

    public static final CANBus CLAW_CANBUS =  new CANBus("");
    public static final int CAN_RANGE_ID = 17;

    public static final Integer SHOOTER_MOTOR_ID = 1;
    public static final Double SHOOTER_GEAR_RATIO = 2.0;

    public static final Double SHOOTER_SPEED = 0.15;
    public static final Double SHOOTER_SLOW_SPEED = 0.125;
    public static final Double SHOOTER_MOTOR_MAX_RPM = 6784.0;




}
