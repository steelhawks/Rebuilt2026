package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.util.Units;
import org.steelhawks.util.TunableNumber;

public class IntakeConstants {


    public static final int EXTENSION_LEFT_MOTOR_ID = 0;
    public static final int EXTENSION_RIGHT_MOTOR_ID = 1;
    public static final TunableNumber EXTENSION_GEAR_RATIO = new TunableNumber("Extension Gear Ratio", 4 / 1);
    public static final String EXTENSION_CANBUS_NAME = "HI";

    public static final int EXTENSION_LEFT_ENCODER_ID = 1;
    public static final int EXTENSION_RIGHT_ENCODER_ID = 2;

    public static final int ROLLER_MOTOR_ID = 0;
    public static final int ROLLER_ENCODER_ID = 0;

    public static final double PINION_RADIUS = Units.inchesToMeters(1.033922 / 2.0);
    public static final double PINION_DIAMETER = PINION_RADIUS / 2.0;
    public static final double PINION_CIRCUMFERENCE = PINION_DIAMETER * Math.PI;
    public static final double PINION_ROTATION = PINION_CIRCUMFERENCE / EXTENSION_GEAR_RATIO.getAsDouble();
    public static final double PINION_METERS_TO_RADIANS = PINION_RADIUS / EXTENSION_GEAR_RATIO.getAsDouble();

    public static final double EXTENSION_ANGLE = Units.degreesToRadians(15.0);

    public static final TunableNumber EXTENSION_CURRENT_LIMIT = new TunableNumber("Extension Current Limit", 0.0);

    public static final TunableNumber ROLLER_CURRENT_LIMIT = new TunableNumber("Roller Current Limit", 0.0);

    public static double MIN_EXTENSION = 0.0;
    public static double MAX_EXTENSION = Units.inchesToMeters(24.0);
    public static double EXTENSION_KG = 21.3;


    public static TunableNumber MAX_VELOCITY_PER_SEC = new TunableNumber("Velocity Max", 0.0);
    public static TunableNumber MAX_ACCEL_METERS_PER_SEC = new TunableNumber("Accel Max", 0.0);

    public static double INTAKE_SPEED = 0.0;
    public static double EJECT_SPEED = -0.0;

    public static final double INTAKE_VELOCITY_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(3000.0);
    public static final double EJECT_VELOCITY_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(-2000.0);

    public static final TunableNumber EXTENSION_POSITION_KS = new TunableNumber("Extension Position KS", 0.0);
    public static final TunableNumber EXTENSION_POSITION_KG = new TunableNumber("Extension Position KG", 0.0);

    public static final TunableNumber EXTENSION_VELOCITY_KV = new TunableNumber("Extension Velocity KV", 0.0);


    public static final TunableNumber ROLLER_KS = new TunableNumber("Roller KS", 0.0);
    public static final TunableNumber ROLLER_KV = new TunableNumber("Roller KV", 0.0);

    public static final double POSITION_TOLERANCE = 0.02;


}
