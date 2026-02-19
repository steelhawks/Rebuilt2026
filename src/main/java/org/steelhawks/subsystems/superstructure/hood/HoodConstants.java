package org.steelhawks.subsystems.superstructure.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.steelhawks.util.LoggedTunableNumber;

public class HoodConstants {
    public static final int MOTOR_ID = 0;
    public static final int CANCODER_ID = 0;
    public static final double REDUCTION = 4.357 / 1.0; // or 3.357 i dont remmber

    public static final double M = Units.lbsToKilograms(0.0);
    public static final double G = 9.81;
    public static final double R = Units.inchesToMeters(0.0); // dist from pivot point to CoM
    public static final double kT = DCMotor.getKrakenX44Foc(1).KtNMPerAmp;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Hood/kI", 0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS", 0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG", (M * G * R) / (kT * REDUCTION));
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA", 0);

    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(30.0);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(70.0);

    public static final Rotation2d MAG_OFFSET = Rotation2d.fromRotations(0.0).plus(MIN_ANGLE);
    public static final double TOLERANCE = 0.02;
}
