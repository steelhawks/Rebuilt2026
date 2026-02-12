package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.util.Units;
import org.steelhawks.util.TunableNumber;

public class IntakeConstants {


    public static final int EXTENSION_LEFT_MOTOR_ID = 0;
    public static final int EXTENSION_RIGHT_MOTOR_ID = 1;
    public static final TunableNumber EXTENSION_GEAR_RATIO = new TunableNumber("Extension Gear Ratio", 4 / 1);

    public static final double ROLLER_MOTOR_ID = 0;

    public static final double PINION_RADIUS = Units.inchesToMeters(1.033922 / 2.0);
    public static final double PINION_DIAMETER = PINION_RADIUS / 2.0;
    public static final double PINION_CIRCUMFERENCE = PINION_DIAMETER * Math.PI;
    public static final double PINION_ROTATION = PINION_CIRCUMFERENCE / EXTENSION_GEAR_RATIO.getAsDouble();







}
