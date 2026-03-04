package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

public class ShooterConstants {

    public static final class Hood {
        public static final double REDUCTION = 4.357 / 1.0; // or 3.357 i dont remmber

        public static final double M = Units.lbsToKilograms(0.0);
        public static final double G = 9.81;
        public static final double R = Units.inchesToMeters(0.0); // dist from pivot point to CoM
        public static final double kT = DCMotor.getKrakenX44Foc(1).KtNMPerAmp;

        public static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG", (M * G * R) / (kT * REDUCTION));

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(30.0); // min angle is full extension
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(70.0); // max angle is home

        public static final Rotation2d MAG_OFFSET = Rotation2d.fromRotations(0.0).plus(MIN_ANGLE);
        public static final double TOLERANCE = 0.02;
    }

    public static final class Turret {
//        public static final double MOTOR_REDUCTION =
//            Constants.value(0.0, 200.0 / 20.0, 0.0, (18.0 / 18.0) * (46.0 / 18.0) * (96.0 / 12.0));
//        public static final double ENCODER_REDUCTION = (16.0 / 96.0) * (28.0 / 16.0) * (64.0 / 16.0);
//        public static final double CURRENT_LIMIT = 45.0;
    }
}
