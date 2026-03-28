package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;
import org.steelhawks.*;
import org.steelhawks.subsystems.superstructure.flywheel.FlywheelIOInputsAutoLogged;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.ArrayList;
import java.util.List;
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

    private static final Translation2d BLUE_HUB = new Translation2d(
            Units.inchesToMeters(121.30),   // 3.081m from blue wall
            Units.inchesToMeters(158.845)   // 4.035m — field centerline
    );

    private static final Translation2d RED_HUB = new Translation2d(
            Units.inchesToMeters(529.92),   // 651.22 - 121.30, mirrored
            Units.inchesToMeters(158.845)
    );


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


    private double angleToHub(Pose2d pose) {
        Translation2d hub = AllianceFlip.apply(FieldConstants.HUB_CENTER_3D.toTranslation2d());
        var turretTranslation = new Pose3d(pose).transformBy(Constants.RobotConstants.ROBOT_TO_TURRET).toPose2d().getTranslation();
        var direction = hub.minus(turretTranslation);
        var findRelativeAngle = direction.getAngle().getRotations();
        double turretRelativeAngle = MathUtil.angleModulus(
                findRelativeAngle - pose.getRotation().getRadians());
        desiredRotation = turretAngle(turretRelativeAngle, inputs.position.getRotations());

        return desiredRotation.getRotations();
    }

    private void dead_zone(double currentAngle) {

    }

    private Rotation2d turretAngle(double targetAngle, double currentAngle) {
        targetAngle = MathUtil.angleModulus(targetAngle);
        double bestAngle = currentAngle;
        double smallestMove = Double.POSITIVE_INFINITY;
        double[] candidates = {
                targetAngle,
                targetAngle + 2 * Math.PI,
                targetAngle - 2 * Math.PI,
        };
        for (double candidate : candidates) {
            if (candidate >= constants.minRotation().getRadians() && candidate <= constants.maxRotation().getRadians()) {
                double move = Math.abs(candidate - currentAngle);
                if (move < smallestMove) {
                    smallestMove = candidate;
                    bestAngle = candidate;
                }
            }
        }

        if (smallestMove == Double.POSITIVE_INFINITY) {
            bestAngle = MathUtil.clamp(targetAngle,
                    constants.minRotation().getRadians(),
                    constants.maxRotation().getRadians());
        }

        return Rotation2d.fromRadians(bestAngle);
    }

    private double createTrajectory(Translation2d target2d, Translation3d target3d) {
        List<Translation3d> trajectoryPoints = new ArrayList<>();
        int numPoints = 50;
        var robot = getPose();
        var turretTranslation = new Pose3d(robot)
                .transformBy(Constants.RobotConstants.ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation();
        var direction = target2d.minus(turretTranslation);
        double fieldRelativeAngle = direction.getAngle().getRadians();
        /* shooter structure, for now skip everything here */
//        var projectileData = ShooterStructure.Static.calculateShot(target3d, target3d);
//        if (projectileData == null) {
//            return new ArrayList<>();
//        }
//        double launchAngle = projectileData.hoodAngle();
//        double timeOfFlight = ShooterStructure.calculateTimeofFlight(
//                projectileData.exitVelocity(),
//                launchAngle,
//                turretTranslation.getDistance(target2d)
//        );
//
//        for (int i = 0; i <= numPoints; i++) {
//            double t = (timeOfFlight / numPoints) * i;
//            double x = projectileData.exitVelocity() * Math.cos(launchAngle) * t;
//            double y = projectileData.exitVelocity() * Math.sin(launchAngle) * t
//                    - 0.5 * 9.81 * t * t;
//            // to field relative
//            double fieldX = turretTranslation.getX() + x * Math.cos(fieldRelativeAngle);
//            double fieldY = turretTranslation.getY() + x * Math.sin(fieldRelativeAngle);
//            double fieldZ = Constants.RobotConstants.ROBOT_TO_TURRET.getZ() + y;
//            trajectoryPoints.add(new Translation3d(fieldX, fieldY, fieldZ));
//        }

        return Units.degreesToRadians(fieldRelativeAngle);
    }

    private TrapezoidProfile buildProfile() {
        return new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())
        );
    }

    public boolean atGoal() {
        return Math.abs(inputs.position.getRotations() - goal.position) < POSITION_TOLERANCE_ROTATIONS;
    }

    private Pose2d getPose() {
        return poseSupplier != null ? poseSupplier.get() : new Pose2d();
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
            Create Helper functions to find angle to hub, find current angle of turret, dead zone manager function.
            Implement functions in both tracking and dead zone
        */

        if (shouldRun) {
            switch (turretState) {
                case IDLE ->  {
                    if (!atGoal()) {
                        io.stopTurret();
                        double acceleration = (setpoint.velocity - inputs.velocityRadPerSec.getRotations()) / Constants.UPDATE_LOOP_DT;
                        double feedforward = kS.getAsDouble() * Math.signum(setpoint.velocity) + kA.getAsDouble() * acceleration;
                        io.runTurretPivot(setpoint.position, feedforward);
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
                    if (inputs.position.getRotations() >= constants.maxRotation().getRotations() && inputs.position.getRotations() <= constants.minRotation().getRotations()) {
                       turretState =  TurretState.DEAD_ZONE;
                    } else {
                        switch (trackingState) {
                            case HUB -> {

                            }
                            case NO_HUB ->  {
                            }
                            default -> io.stopTurret();
                        }
                    }
                }
                case DEAD_ZONE -> {

                }
            }
            desiredRotation = Rotation2d.fromRadians(
                    MathUtil.clamp(desiredRotation.getRadians(), constants.minRotation().getRadians(), constants.maxRotation().getRadians()));

        }
    }

}
