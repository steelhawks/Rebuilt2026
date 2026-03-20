package org.steelhawks.subsystems.Superstructure.flywheel;

import org.steelhawks.util.LoggedTunableNumber;

public class FlywheelConstants {

    public static int leftMotorID = 1;
    public static int rightMotorID = 2;

    public static double GEAR_RATIO = ( 2.0 ) / ( 1.0 );

    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;

    public static final double radius = 3.0;

    public double kP() {
        return kP;
    }

    public double kI() {
        return kI;
    }

    public double kD() {
        return kI;
    }
}
