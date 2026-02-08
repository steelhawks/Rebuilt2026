package org.steelhawks;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.util.Maths;
import org.steelhawks.util.LoggedTunableNumber;


public final class Constants {

    public static final int POWER_DISTRIBUTION_CAN_ID =
        getRobot() == RobotType.ALPHABOT
            ? 0
            : 1;
    public static final PowerDistribution.ModuleType PD_MODULE_TYPE =
        getRobot() == RobotType.ALPHABOT
            ? PowerDistribution.ModuleType.kCTRE
            : PowerDistribution.ModuleType.kRev;
    public static final double UPDATE_LOOP_DT = 0.020;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        OMEGABOT,
        ALPHABOT,
        CHASSIS,
        LAST_YEAR,
        TEST_BOARD,
        SIMBOT
    }

    // Change this based on what robot is being used.
    private static final RobotType ROBOT = RobotType.SIMBOT;

    /**
     * The robot type.
     *
     * <p>
     *     To run a physics simulator make sure you set it to RobotType.SIMBOT
     *     </p>If you want to replay a log file set it to the robot type you want to replay and just run the simulator.
     * </p>
     */
    private static final RobotType ROBOT_TYPE =
        isCI() ?
            RobotType.SIMBOT : // set to simbot when doing CI check on GitHub
            ROBOT; // actual mode you want

    private static boolean isCI() {
        return System.getenv("CI") != null;
    }

    public static final String ROBOT_NAME =
        switch (ROBOT) {
            case OMEGABOT -> "OMEGA";
            case ALPHABOT -> "ALPHA";
            case CHASSIS -> "CHASSIS";
            case LAST_YEAR -> "LAST_YEAR";
            case TEST_BOARD -> "TEST_BOARD";
            case SIMBOT -> "Simulation";
        };

    public static Mode getMode() {
        return switch (ROBOT_TYPE) {
            case ALPHABOT, OMEGABOT, CHASSIS, LAST_YEAR, TEST_BOARD ->
                RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

    public static RobotType getRobot() {
        if (RobotBase.isReal() && ROBOT_TYPE == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using omega robot as default.", AlertType.kError)
                .set(true);
            return RobotType.OMEGABOT;
        }

        return ROBOT_TYPE;
    }

    public static CANBus getCANBus() {
        return switch (getRobot()) {
            case OMEGABOT, LAST_YEAR, SIMBOT -> new CANBus("canivore");
            case ALPHABOT, CHASSIS, TEST_BOARD -> new CANBus("");
        };
    }

    public static final class RobotConstants {
        public static final double BAD_BATTERY_THRESHOLD = 11.6;
        public static final double ROBOT_LENGTH_WITH_BUMPERS = Units.inchesToMeters(0.0);

        public static final Transform3d ROBOT_TO_TURRET =
            new Transform3d(0.0, 0.0, Units.inchesToMeters(18.0), new Rotation3d(0.0, 0.0, 0.0));

        public static final double FIXED_SHOOTER_ANGLE = Math.toRadians(45.0);

        public static final Rotation2d MIN_HOOD_ANGLE = Rotation2d.fromDegrees(30.0);
        public static final Rotation2d MAX_HOOD_ANGLE = Rotation2d.fromDegrees(70.0);
        public static final double ANGLE_INCREMENT = Units.degreesToRadians(0.25);
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class Deadbands {
        public static final double DRIVE_DEADBAND = 0.3;
    }

    public static final class LEDConstants {
        public static final int PORT;
        public static final int LENGTH;

        static {
            switch (getRobot()) {
                case ALPHABOT, LAST_YEAR -> {
                    PORT = 0;
                    LENGTH = 40;
                }
                default -> {
                    PORT = 0;
                    LENGTH = 80;
                }
            }
        }
    }

    @SuppressWarnings("ConstantConditions")
    public static final class AutonConstants {
        public static final LoggedTunableNumber TRANSLATION_KP = new LoggedTunableNumber("Swerve/TranslationkP", Constants.omega(5.0, 5.0));
        public static final LoggedTunableNumber TRANSLATION_KI = new LoggedTunableNumber("Swerve/TranslationkI", Constants.omega(0.0, 0.0));
        public static final LoggedTunableNumber TRANSLATION_KD = new LoggedTunableNumber("Swerve/TranslationkD", Constants.omega(0.1, 0.1));

        public static final LoggedTunableNumber ROTATION_KP = new LoggedTunableNumber("Swerve/RotationkP", Constants.omega(5.0, 5.0));
        public static final LoggedTunableNumber ROTATION_KI = new LoggedTunableNumber("Swerve/RotationkI", Constants.omega(0.0, 0.0));
        public static final LoggedTunableNumber ROTATION_KD = new LoggedTunableNumber("Swerve/RotationkD", Constants.omega(0.1, 0.1));

        public static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("Swerve/AnglekP", Constants.omega(1.0, 2.5));
        public static final LoggedTunableNumber ANGLE_KI = new LoggedTunableNumber("Swerve/AnglekI", Constants.omega(0.0, 0.0));
        public static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("Swerve/AnglekD", Constants.omega(0.0, 1.0));

        // Pathfinder
        public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.omega(5.0, 0.0);
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = Constants.omega(5.5, 0.0);
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.omega(Units.degreesToRadians(540.0), 0.0);
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Constants.omega(Units.degreesToRadians(920.0), 0.0);

        public static final PathConstraints CONSTRAINTS = new PathConstraints(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param hawkrider The HawkRider constant value
     * @param alpha The Alpha constant value
     * @param omega The Omega constant value
     * @param sim The simulation constant value
     * @return The correct constant
     */
    public static <T> T value(T hawkrider, T alpha, T chassis, T omega, T sim) {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> alpha;
            case CHASSIS -> chassis;
            case OMEGABOT -> omega;
            case LAST_YEAR -> hawkrider;
            case SIMBOT -> sim;
            default -> null;
        };
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param hawkrider The HawkRider constant value
     * @param alpha The Alpha constant value
     * @param omega The Omega constant value
     * @return The correct constant
     */
    public static <T> T value(T hawkrider, T alpha, T chassis, T omega) {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> alpha;
            case CHASSIS -> chassis;
            case OMEGABOT, SIMBOT -> omega;
            case LAST_YEAR -> hawkrider;
            default -> null;
        };
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     * In SIMBOT, the omega constant is used.
     *
     * @param alpha The Alpha constant value
     * @param omega The Omega constant value
     * @return The correct constant
     */
    public static <T> T value(T alpha, T omega) {
        return switch (Constants.getRobot()) {
            case ALPHABOT -> alpha;
            case OMEGABOT, SIMBOT -> omega;
            default -> null;
        };
    }

    /**
     * Automatically returns the correct constant based on which robot type it is running on.
     * This utility does not send the simulation value for replay mode, making replay mode work properly.
     *
     * @param omega The Omega constant value
     * @param sim The sim constant value
     * @return The correct constant
     */
    public static <T> T omega(T omega, T sim) {
        return switch (Constants.getRobot()) {
            case OMEGABOT -> omega;
            case SIMBOT -> sim;
            default -> null;
        };
    }

    public static double omega(double omega, double sim) {
        return switch (Constants.getRobot()) {
            case OMEGABOT -> omega;
            case SIMBOT -> sim;
            default -> 0.0;
        };
    }

    public static <T> T requireNonNullConst(T obj) {
        if (obj == null) {
            DriverStation.reportWarning(
                "Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.", false);
            throw new IllegalCallerException(
                "\"Robot chosen does not have this constant configured. Please null this subsystem if this was intentional.\"");
        }
        return obj;
    }

    public static <T> T loggedValue(String key, T obj) {
        requireNonNullConst(obj);
        if (Toggles.debugMode.get()) {
            Logger.recordOutput("Debug/" + key, obj.toString());
        }
        return obj;
    }

    /**
     * Automatically returns a Transform3d that correctly converts the Onshape coordinate system into the WPILib coordinate system.
     * <br>
     * <br>
     * Learn more <a href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html">here</a> about the WPILib coordinate system.
     * @param x From Onshape
     * @param y From Onshape
     * @param z From Onshape
     * @param roll From Onshape
     * @param pitch From Onshape
     * @param yaw From Onshape
     * @return The WPILib coordinate system ready Transform3d
     */
    public static Transform3d fromOnshapeCoordinates(
        double x, double y, double z,
        double roll, double pitch, double yaw
    ) {
        return new Transform3d(
            Units.inchesToMeters(y),
            Units.inchesToMeters(-x),
            Units.inchesToMeters(-z),
            new Rotation3d(
                Units.degreesToRadians(roll),
                Units.degreesToRadians(pitch),
                Units.degreesToRadians(yaw)
            )
        );
    }

    /**
     * Automatically logs a Translation2d as a Pose2d so that it can be viewed in AdvantageScope as a point
     *
     * @param name Name of the logged Translation2d.
     * @param translation The Translation2d coordinate to be shown.
     */
    public static void toLoggedPoint(String name, Translation2d translation) {
        Logger.recordOutput("Coordinate/" + name, Maths.pose2dFromTranslation(translation));
    }
}
