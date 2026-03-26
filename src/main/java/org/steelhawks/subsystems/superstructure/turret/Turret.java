package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOInputsAutoLogged;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.Supplier;

public class Turret extends SubsystemBase {
    private TurretIO io;
    private TurretIOInputsAutoLogged inputs;
    private TurretState turretState = TurretState.IDLE;

    private static final double POSITION_TOLERANCE_ROTATIONS = 0.01;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal =  new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kA;

    private final LoggedTunableNumber maxVelocity;
    private final LoggedTunableNumber maxAcceleration;
    private final LoggedTunableNumber currentThreshold;

    private Rotation2d desiredRotation = new Rotation2d();
    private double homingVolts = 0.1;
    private Supplier<Pose2d> poseSupplier;

    private BuilderConstants.TurretConstants constants;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;


    enum TurretState {
        IDLE,
        HOMED,
        ROTATING,
        ZEROED,
    }

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, BuilderConstants.TurretConstants constants) {
        this.io = io;
        this.constants = constants;
        this.poseSupplier = poseSupplier;

        kP = new LoggedTunableNumber("Turret/kP", constants.kP());
        kI = new LoggedTunableNumber("Turret/kI", constants.kI());
        kD = new LoggedTunableNumber("Turret/kD", constants.kD());
        kS = new LoggedTunableNumber("Turret/kS", constants.kS());
        kA = new LoggedTunableNumber("Turret/kA", constants.kA());

        inputs = new TurretIOInputsAutoLogged();

        maxAcceleration = new LoggedTunableNumber("Turret/maxAcceleration", constants.maxAccelerationRadPerSecSq());
        maxVelocity = new LoggedTunableNumber("Turret/MaxVelocity", constants.maxVelocityRadPerSec());
        currentThreshold = new LoggedTunableNumber("Turret/CurrentHomingThreshold", 4);

        profile = buildProfile();

    }

    private TrapezoidProfile buildProfile() {
        return new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())
        );
    }

    public boolean atGoal() {
        return Math.abs(inputs.position.getRotations() - goal.position) < POSITION_TOLERANCE_ROTATIONS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/State", turretState.toString());

        final boolean shouldRun = Toggles.Turret.isEnabled.get();

        if (Toggles.tuningMode.getAsBoolean()) {

            LoggedTunableNumber.ifChanged(
                    hashCode(), () -> io.setTurretPID(kP.get(), kI.get(), kD.get()), kP, kI, kD
            );

            LoggedTunableNumber.ifChanged(
                    hashCode(), () -> profile = buildProfile(), maxVelocity, maxAcceleration
            );

            if (Toggles.Turret.toggleVoltageOverride.getAsBoolean()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Turret/TuningVolts", 0.0);
                }
                io.runOpenLoop(tuningVolts.getAsDouble(), false);
            }

            if (Toggles.Turret.toggleCurrentOverride.getAsBoolean()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Turret/TuningAmps", 0.0);
                }
                io.runOpenLoop(tuningAmps.getAsDouble(), false);
            }
        }

        if (shouldRun) {

        } else {
            io.stopTurret();
            return;
        }
    }

}
