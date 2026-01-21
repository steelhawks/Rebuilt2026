package org.steelhawks.subsystems.TurretSuperStructure.turret;

import com.ctre.phoenix6.CANBus;
import org.steelhawks.util.LoggedTunableNumber;

public class TurretConstants {

    public static int MOTOR_ID = 1;
    public static CANBus TURRET_CANBUS = new CANBus("");
    public static Double GEAR_RATIO = 2.0 / 1.0 / 1.0;

    public static Double TURRET_RUN_SPEED = 0.15;

    public static final LoggedTunableNumber SHOOTER_KP = new LoggedTunableNumber("SHOOTER_KP", 2.4);
    public static final LoggedTunableNumber SHOOTER_KI = new LoggedTunableNumber("SHOOTER_KI", 0);
    public static final LoggedTunableNumber SHOOTER_KD = new LoggedTunableNumber("SHOOTER_KP", 0.1);
    public static final LoggedTunableNumber SHOOTER_KS = new LoggedTunableNumber("SHOOTER_KS", 0.2);
    public static final LoggedTunableNumber SHOOTER_KV = new LoggedTunableNumber("SHOOTER_KV", 0.16);
    public static final LoggedTunableNumber SHOOTER_KA = new LoggedTunableNumber("SHOOTER_KV", 0);

    public static Double TURRET_MAX_RPM = 6842.0;

}
