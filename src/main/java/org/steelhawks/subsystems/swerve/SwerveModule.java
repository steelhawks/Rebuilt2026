package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Toggles;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.generated.TunerConstantsAlpha;
import org.steelhawks.generated.TunerConstantsLastYear;
import org.steelhawks.util.BatteryUtil;
import org.steelhawks.util.LoggedTunableNumber;

public class SwerveModule {

    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Swerve/DrivekP");
    private static final LoggedTunableNumber drivekI = new LoggedTunableNumber("Swerve/DrivekI");
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Swerve/DrivekD");

    private static final LoggedTunableNumber steerkP = new LoggedTunableNumber("Swerve/SteerkP");
    private static final LoggedTunableNumber steerkI = new LoggedTunableNumber("Swerve/SteerkI");
    private static final LoggedTunableNumber steerkD = new LoggedTunableNumber("Swerve/SteerkD");

    private final LoggedTunableNumber driveOpenLoop = new LoggedTunableNumber("Swerve/DriveOpenLoop", 0.0);
    private final LoggedTunableNumber turnOpenLoop = new LoggedTunableNumber("Swerve/TurnOpenLoop", 0.0);

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final SwerveModuleConstants<
        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        constants;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};
    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    public SwerveModule(
        ModuleIO io,
        int index,
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constants) {
        this.io = io;
        this.index = index;
        this.constants = constants;

        drivekP.initDefault(constants.DriveMotorGains.kP);
        drivekI.initDefault(constants.DriveMotorGains.kI);
        drivekD.initDefault(constants.DriveMotorGains.kD);

        steerkP.initDefault(constants.SteerMotorGains.kP);
        steerkI.initDefault(constants.SteerMotorGains.kI);
        steerkD.initDefault(constants.SteerMotorGains.kD);

        driveDisconnectedAlert =
            new Alert(
                "Disconnected drive motor on module " + index + ".",
                AlertType.kError);
        turnDisconnectedAlert =
            new Alert(
                "Disconnected turn motor on module " + index + ".", AlertType.kError);
        turnEncoderDisconnectedAlert =
            new Alert(
                "Disconnected turn encoder on module " + index + ".",
                AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + index, inputs);
        BatteryUtil.recordCurrentUsage(inputs.driveCurrentAmps + inputs.turnCurrentAmps);

        // calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);

        if (Toggles.tuningMode.get()) {
            if (Toggles.Swerve.driveOpenLoopOverride.get()) {
                io.setDriveOpenLoop(driveOpenLoop.get());
            }
            if (Toggles.Swerve.turnOpenLoopOverride.get()) {
                io.setTurnOpenLoop(turnOpenLoop.get());
            }
            LoggedTunableNumber.ifChanged(hashCode(), () -> {
                io.setDrivePID(drivekP.get(), drivekI.get(), drivekD.get());
                io.setSteerPID(steerkP.get(), steerkI.get(), steerkD.get());
            }, drivekP, drivekI, drivekD, steerkP, steerkI, steerkD);
        }
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
        io.setTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /**
     * Characterizes the turn motor feedforward.
     */
    public void runTurnCharacterization(double output) {
        io.setDriveOpenLoop(0);
        io.setTurnOpenLoop(output);
    }

    /**
     * Characterizes the robot's angular motion.
     * Useful for finding the Moment of Inertia.
     */
    public void runAngularCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(
            new Rotation2d(constants.LocationX, constants.LocationY)
                .plus(Rotation2d.kCCW_Pi_2));
    }

    /**
     * Disables all outputs to motors.
     */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getPositionMeters() {
        return inputs.drivePositionRad * constants.WheelRadius;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * constants.WheelRadius;
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /**
     * Returns the module positions received this cycle.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /**
     * Returns the timestamps of the samples received this cycle.
     */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /**
     * Returns the module position in radians.
     */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /**
     * Returns the module velocity in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    /**
     * Returns if the drive motors are stalling, up against a wall
     */
    public boolean getIsStalling() {
        double stallCurrent =
            switch (Constants.getRobot()) {
                case ALPHABOT -> TunerConstantsAlpha.FrontLeft.SlipCurrent;
                case LAST_YEAR -> TunerConstantsLastYear.FrontLeft.SlipCurrent;
                default -> TunerConstants.FrontLeft.SlipCurrent;
            };
        return inputs.driveCurrentAmps >= stallCurrent;
    }
}
