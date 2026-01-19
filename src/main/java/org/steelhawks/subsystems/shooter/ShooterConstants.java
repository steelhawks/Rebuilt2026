package org.steelhawks.subsystems.shooter;


import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.Constants;
import org.steelhawks.util.LoggedTunableNumber;

public class ShooterConstants extends SubsystemBase {

    public static final CANBus CAN_NAME = new CANBus("canivore");
    public static final int SHOOTER_MASTERMOTOR_ID_OMEGA = 0;
    public static final int SHOOTER_FOLLOWERMOTOR_ID_OMEGA = 1;
    public static final int SHOOTER_MOTOR_ID_ALPHA = 2;

    public static final double TOLERANCE = 1.0;
    public static final double GEAR_REDUCTION = 2.0;
    public static final double FLYWHEEL_RADIUS = Units.inchesToMeters(2.0);

    public static final double MAX_RPM = 0.0;
    public static final double MAX_VELOCITY_ROTPERSEC = MAX_RPM / 60;
    public static final double MAX_VELOCITY_METERPERSEC = (FLYWHEEL_RADIUS * 2 * Math.PI * MAX_VELOCITY_ROTPERSEC);

    public static final double MAX_ACCELERATION_ROTPERSEC_2 = 0.0;

    public static final LoggedTunableNumber kP =
        new LoggedTunableNumber("kP", Constants.omega(0, 0));
    public static final LoggedTunableNumber kI =
        new LoggedTunableNumber("kI", Constants.omega(0, 0));
    public static final LoggedTunableNumber kD =
        new LoggedTunableNumber("kD", Constants.omega(0, 0));

    public static final LoggedTunableNumber kS =
        new LoggedTunableNumber("kS", Constants.omega(0, 0));
    public static final LoggedTunableNumber kV =
        new LoggedTunableNumber("kV", Constants.omega(0, 0));
    }



