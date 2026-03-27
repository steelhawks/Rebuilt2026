package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOInputsAutoLogged;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.Supplier;

public class Turret extends SubsystemBase {
    private TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private TurretState turretState =  TurretState.IDLE;
    private TrackingState trackingState = TrackingState.NO_HUB;

    private static final double POSITION_TOLERANCE_ROTATIONS = 0.01;

    private TrapezoidProfile profile;
    private final TrapezoidProfile.State goal =  new TrapezoidProfile.State();
    private final TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kA;

    private final LoggedTunableNumber maxVelocity;
    private final LoggedTunableNumber maxAcceleration;
    private final LoggedTunableNumber currentThreshold;

    private Rotation2d desiredRotation = new Rotation2d();
    private LoggedTunableNumber homingVolts;
    private Supplier<Pose2d> poseSupplier;

    private BuilderConstants.TurretConstants constants;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;


    enum TurretState {
        IDLE,
        HOMED,
        TRACKING,
        DEAD_ZONE
    }

    enum TrackingState {
        HUB,
        NO_HUB
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

    // Helper Functions



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

        if (shouldRun) {
            if (turretState != TurretState.HOMED) {
                profile = new  TrapezoidProfile(new TrapezoidProfile.Constraints(Math.PI, 0.0));
                desiredRotation = new Rotation2d(Math.PI);
                goal.position = desiredRotation.getRadians();
                turretState = TurretState.HOMED;
            }
        }

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

            if (Toggles.Turret.toggleHomingVoltageOverride.getAsBoolean()) {
                if (homingVolts == null) {
                    homingVolts = new LoggedTunableNumber("Turret/HomingVolts", 0.0);
                }
                io.runOpenLoop(homingVolts.getAsDouble(), false); // Till you get to desired rotation
                setpoint.position = inputs.encoderPosition.getRadians();
                setpoint.velocity = inputs.velocityRadPerSec.getRadians();
            }

            if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
                profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
            }
        }
        /* Overall Goal: Have turret constantly track the hub, moving around counter-clockwise and updating its state from These States:
         HOMING (where it's at the Math.PI setpoint and ready to start tracking)
         TRACKING (This should be the default turret state after homing)
         DEAD_ZONE (the point in time where the turret is locked and has to rotate -direction in order to get back to homing state. Dead zone should happen when the
         target is out of reach from the direction that you're currently going in (360 degrees))
         IDLE (Not on, should be default before homed)
        *
        * The turret itself should track the hub through vision system being at the angle of the hub, so have a separate vision state that's either:
        HUB (the robot sees the hub and constantly tracks angle as robot moves by referencing current angle with vision angle)
        NO_HUB (The robot default, the current turret angle and vision angle are either not known or non-existent.)
        *
        * Switch the Turret Cases and in the Tracking case have the TrackingState available, then in the dead_zone turn the opposite way until dead one again, or
        until its homed.


        Steps:
            Modify the TurretState and code currently relating to the state
            Create the TrackingState
            Start Helper Functions for calculating angles and turret dead zones
        */

        if (shouldRun) {
            switch (turretState) {
                case IDLE ->  {
                    if (!atGoal()) {
                        io.stopTurret();
                        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Math.PI, 0.0));
                        desiredRotation = new Rotation2d(Math.PI);
                        goal.position = desiredRotation.getRadians();
                        io.setTurretPosition(desiredRotation.getRadians());
                    }
                    turretState = TurretState.HOMED;
                }
                case HOMED -> {
                    // check if it's homed
                    io.setTurretPosition(goal.position);
                    io.stopTurret();
                    turretState = TurretState.TRACKING;
                }
                case TRACKING -> {
                    // TRACKING STATE SWITCH CASE FOR HUB AND NO HUB DATA. CONSTANTLY CHECK FOR EVERYTHING.
                    // Constantly rotate unless hit dead zone.
                    if (setpoint.position > goal.position || setpoint.position > inputs.encoderPosition.getRadians()) {
                       turretState =  TurretState.DEAD_ZONE;
                    }
                }
                case DEAD_ZONE -> {

                }
            }
            desiredRotation = Rotation2d.fromRadians(
                    Math.clamp(desiredRotation.getRadians(), constants.minRotation().getRadians(), constants.maxRotation().getRadians()));

        }
    }

}
