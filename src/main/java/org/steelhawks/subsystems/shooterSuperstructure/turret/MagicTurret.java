package org.steelhawks.subsystems.shooterSuperstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.subsystems.shooterSuperstructure.ShooterStructure;
import org.steelhawks.subsystems.superstructure.FuelStateTracker;
import org.steelhawks.util.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MagicTurret extends SubsystemBase {
    private final static double FF_RAMP_RATE = 5.0;

    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;

    private final FuelStateTracker fuelStateTracker = new FuelStateTracker();

    // kV here scales the geometry-based velocity FF (calculateTurretVelocityFF),
    // NOT the motor model — that lives in Slot0.kV on the TalonFX
    private static LoggedTunableNumber maxVelocityRadPerSec;
    private static LoggedTunableNumber maxAccelerationRadPerSecSq;
    private static LoggedTunableNumber jerk;
    private static LoggedTunableNumber manualIncrement;
    private static final LoggedTunableNumber tolerance =
            new LoggedTunableNumber("Turret/Tolerance", Units.degreesToRadians(3.0));

    private static LoggedTunableNumber currentHomingThres;
    private static final double homingVolts = 0.1;

    private final Debouncer homingDebouncer = new Debouncer(0.15, DebounceType.kRising);
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final Supplier<Pose2d> poseSupplier;
    private final TurretIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private int trajectoryLoopCounter = 0;
    private static final int TRAJECTORY_LOG_INTERVAL = 5;

    private double manualGoalRad = 0.0;
    private Rotation2d desiredRotation = new Rotation2d();

    // No more TrapezoidProfile, setpoint, or goal state objects
    private boolean brakeModeEnabled = false;
    private boolean isManual = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;
    private DoubleSupplier joystickAxis = null;
    private final SubsystemConstants.TurretConstants constants;

    public MagicTurret(TurretIO io, Supplier<Pose2d> poseSupplier, SubsystemConstants.TurretConstants constants) {
        this.poseSupplier = poseSupplier;
        this.io = io;
        this.constants = constants;
        kP = new LoggedTunableNumber("Turret/kP", constants.kP());
        kI = new LoggedTunableNumber("Turret/kI", constants.kI());
        kD = new LoggedTunableNumber("Turret/kD", constants.kD());
        kS = new LoggedTunableNumber("Turret/kS", constants.kS());
        kV = new LoggedTunableNumber("Turret/kV", 0.0);
        kA = new LoggedTunableNumber("Turret/kA", constants.kA());
        maxVelocityRadPerSec =
                new LoggedTunableNumber("Turret/MaxVelocityRadPerSec", constants.maxVelocityRadPerSec());
        maxAccelerationRadPerSecSq =
                new LoggedTunableNumber("Turret/MaxAccelerationRadPerSecSq", constants.maxAccelerationRadPerSecSq());
        jerk = new LoggedTunableNumber("Turret/Jerk", constants.maxAccelerationRadPerSecSq() * 10.0);
        manualIncrement =
                new LoggedTunableNumber("Turret/ManualIncrement", constants.manualIncrement());
        currentHomingThres =
                new LoggedTunableNumber("Turret/CurrentHomingThresholdAmps", constants.currentHomingThreshold());

        isHomed = Constants.getRobot().equals(RobotType.OMEGABOT);
    }

    private Pose2d getPose() {
        return poseSupplier != null ? poseSupplier.get() : new Pose2d();
    }

    private Rotation2d getPosition() {
        return inputs.positionRad;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    @AutoLogOutput(key = "Turret/AtGoal")
    public boolean atGoal() {
        return atGoal;
    }

    private Rotation2d findBestTurretAngle(double targetAngle, double currentAngle) {
        targetAngle = MathUtil.angleModulus(targetAngle);
        double bestAngle = currentAngle;
        double smallestMove = Double.POSITIVE_INFINITY;
        double[] candidates = {
                targetAngle,
                targetAngle + 2 * Math.PI,
                targetAngle - 2 * Math.PI
        };
        for (double candidate : candidates) {
            if (candidate >= constants.minRotation().getRadians() &&
                    candidate <= constants.maxRotation().getRadians()) {
                double moveDistance = Math.abs(candidate - currentAngle);
                if (moveDistance < smallestMove) {
                    smallestMove = moveDistance;
                    bestAngle = candidate;
                }
            }
        }
        if (smallestMove == Double.POSITIVE_INFINITY) {
            bestAngle = MathUtil.clamp(
                    targetAngle,
                    constants.minRotation().getRadians(),
                    constants.maxRotation().getRadians());
        }
        return Rotation2d.fromRadians(bestAngle);
    }

    // Geometry-based velocity FF — accounts for robot motion relative to target.
    // This is NOT the motor model kV. It stays as withFeedForward() on the MM request.
    private double calculateTurretVelocityFF(Translation2d target2d) {
        if (target2d == null) return 0.0;
        var robot = getPose();
        var turretPos = new Pose3d(robot)
                .transformBy(RobotConstants.ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation();
        var chassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        RobotContainer.s_Swerve.getChassisSpeeds(),
                        RobotState.getInstance().getRotation());
        double omegaRobot = chassisSpeeds.omegaRadiansPerSecond;
        Translation2d robotToTurret = turretPos.minus(robot.getTranslation());
        double turretVx = chassisSpeeds.vxMetersPerSecond - omegaRobot * robotToTurret.getY();
        double turretVy = chassisSpeeds.vyMetersPerSecond + omegaRobot * robotToTurret.getX();
        Translation2d turretVelocity = new Translation2d(turretVx, turretVy);
        Translation2d mrR = target2d.minus(turretPos);
        double distance = mrR.getNorm();
        Translation2d rHat = mrR.div(distance);
        Translation2d rHatPerpendicular = rHat.rotateBy(Rotation2d.kCCW_Pi_2);
        double tangentialVelocity = turretVelocity.dot(rHatPerpendicular);
        return (tangentialVelocity / distance) - omegaRobot;
    }

    private List<Translation3d> createTrajectory(Translation3d target3d, Translation2d target2d) {
        List<Translation3d> trajectoryPoints = new ArrayList<>();
        int numPoints = 50;
        var robot = getPose();
        var turretTranslation = new Pose3d(robot)
                .transformBy(RobotConstants.ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation();
        var direction = target2d.minus(turretTranslation);
        double fieldRelativeAngle = direction.getAngle().getRadians();
        var projectileData = ShooterStructure.Static.calculateShot(target3d, target3d);
        if (projectileData == null || ShooterStructure.isNoSolution(projectileData)) {
            return new ArrayList<>();
        }
        double launchAngle = projectileData.hoodAngle();
        double timeOfFlight = ShooterStructure.calculateTimeOfFlight(
                projectileData.exitVelocity(),
                launchAngle,
                turretTranslation.getDistance(target2d),
                target3d.getZ() - RobotConstants.ROBOT_TO_TURRET.getZ());
        for (int i = 0; i <= numPoints; i++) {
            double t = (timeOfFlight / numPoints) * i;
            double x = projectileData.exitVelocity() * Math.cos(launchAngle) * t;
            double y = projectileData.exitVelocity() * Math.sin(launchAngle) * t
                    - 0.5 * 9.81 * t * t;
            double fieldX = turretTranslation.getX() + x * Math.cos(fieldRelativeAngle);
            double fieldY = turretTranslation.getY() + x * Math.sin(fieldRelativeAngle);
            double fieldZ = RobotConstants.ROBOT_TO_TURRET.getZ() + y;
            trajectoryPoints.add(new Translation3d(fieldX, fieldY, fieldZ));
        }
        Logger.recordOutput("Turret/ProjectileData/Velocity", projectileData.exitVelocity());
        Logger.recordOutput("Turret/ProjectileData/AngleDeg", Math.toDegrees(projectileData.hoodAngle()));
        Logger.recordOutput("Turret/ProjectileData/TimeOfFlight", timeOfFlight);
        return trajectoryPoints;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        BatteryUtil.recordCurrentUsage("Turret", inputs.supplyCurrentAmps);

        // ---- Sim: skip homing ----
        if (Constants.getRobot().equals(RobotType.SIMBOT) && !isHomed && !isZeroed) {
            isHomed = true;
            Logger.recordOutput("Turret/IsHomed", true);
            io.setPosition(0);
            isZeroed = true;
            desiredRotation = Rotation2d.fromRadians(Math.PI);
            Logger.recordOutput("Turret/Zeroed", true);
        }

        // ---- Homing sequence ----
        if (!isHomed && Toggles.Turret.isEnabled.get()) {
            io.runPercentOutput(homingVolts);
            isHomed = homingDebouncer.calculate(
                    inputs.supplyCurrentAmps > currentHomingThres.getAsDouble());
            Logger.recordOutput("Turret/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(-Math.PI / 2);
                io.stop();
                isZeroed = true;
                desiredRotation = Rotation2d.fromRadians(Math.PI);
                Logger.recordOutput("Turret/Zeroed", true);
            }
        }

        // ---- Run gate ----
        final boolean shouldRun =
                DriverStation.isEnabled()
                        && !isManual
                        && ((isHomed && isZeroed) || Constants.getRobot().equals(RobotType.SIMBOT))
                        && Toggles.Turret.isEnabled.get()
                        && (inputs.connected && (inputs.encoderConnected
                        || Constants.getRobot().equals(RobotType.ALPHABOT)))
                        && !Toggles.Turret.toggleVoltageOverride.get()
                        && !Toggles.Turret.toggleCurrentOverride.get()
                        && (getPosition().getRadians() <= constants.maxRotation().getRadians()
                        && getPosition().getRadians() >= constants.minRotation().getRadians());
        Logger.recordOutput("Turret/ShouldRun", shouldRun);

        // ---- Mode transitions ----
        if (DriverStation.isDisabled()) {
            desiredRotation = getPosition();
        }
        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }

        // ---- Live gain + profile updates ----
        if (Toggles.tuningMode.get()) {
            if (Toggles.Turret.toggleVoltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Turret/TuningVolts", 0.0);
                }
                io.runOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Turret.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Turret/TuningAmps", 0.0);
                }
                io.runOpenLoop(tuningAmps.get(), true);
            }
            LoggedTunableNumber.ifChanged(
                    hashCode(),
                    () -> io.setPID(kP.get(), kI.get(), kD.get()),
                    kP, kI, kD);

            // Profile constraint changes push to TalonFX onboard — no profile rebuild needed
            LoggedTunableNumber.ifChanged(
                    hashCode() + 1,
                    () -> io.setMotionMagicConstraints(
                            maxVelocityRadPerSec.get(),
                            maxAccelerationRadPerSecSq.get(),
                            jerk.get()),
                    maxVelocityRadPerSec, maxAccelerationRadPerSecSq, jerk);
        }

        // ---- Manual control ----
        if (isManual && joystickAxis != null) {
            double delta = joystickAxis.getAsDouble() * manualIncrement.getAsDouble();
            manualGoalRad = MathUtil.clamp(
                    manualGoalRad + delta,
                    constants.minRotation().getRadians(),
                    constants.maxRotation().getRadians());
            desiredRotation = Rotation2d.fromRadians(manualGoalRad);
        }

        if (shouldRun) {
            Translation2d velocityTargetFF = null;

            if (Toggles.shooterTuningMode.get()) {
                RobotState.getInstance().setAimState(AimState.TO_HUB);
            }

            // ---- Aim state machine — identical to original ----
            switch (RobotState.getInstance().getAimState()) {
                case TO_HUB -> {
                    fuelStateTracker.updateAtAimstate(AimState.TO_HUB);
                    var robot = getPose();
                    var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
                    var sol = RobotState.getInstance().getMovingShotSolution();
                    if (sol != null && RobotState.getInstance().getShootingState()
                            .equals(ShootingState.SHOOTING_MOVING)) {
                        velocityTargetFF = sol.virtualTarget().toTranslation2d();
                        desiredRotation = findBestTurretAngle(
                                sol.turretAngle().getRadians(),
                                getPosition().getRadians());
                        if (trajectoryLoopCounter % TRAJECTORY_LOG_INTERVAL == 0) {
                            var trajectory = createTrajectory(
                                    hubCenter, sol.virtualTarget().toTranslation2d());
                            Logger.recordOutput("Turret/ScoreTrajectory",
                                    trajectory.toArray(new Translation3d[0]));
                        }
                    } else {
                        velocityTargetFF = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER);
                        var turretTranslation = new Pose3d(robot)
                                .transformBy(RobotConstants.ROBOT_TO_TURRET)
                                .toPose2d()
                                .getTranslation();
                        double fieldRelativeAngle = Math.atan2(
                                hubCenter.getY() - turretTranslation.getY(),
                                hubCenter.getX() - turretTranslation.getX());
                        double turretMountingYaw =
                                RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
                        double turretRelativeAngle = MathUtil.angleModulus(
                                fieldRelativeAngle
                                        - robot.getRotation().getRadians()
                                        - turretMountingYaw);
                        desiredRotation = findBestTurretAngle(
                                turretRelativeAngle, getPosition().getRadians());
                    }
                }
                case FERRY -> {
                    fuelStateTracker.updateAtAimstate(AimState.FERRY);
                    var robot = getPose();
                    var ferryGoal2d = AllianceFlip.apply(
                            FieldConstants.getClosestPointOnLine(
                                    FieldConstants.Ferrying.START_LINE,
                                    FieldConstants.Ferrying.END_LINE));
                    var ferryGoal3d = new Translation3d(
                            ferryGoal2d.getX(), ferryGoal2d.getY(), 0.0);
                    velocityTargetFF = ferryGoal3d.toTranslation2d();
                    var turretTranslation = new Pose3d(robot)
                            .transformBy(RobotConstants.ROBOT_TO_TURRET)
                            .toPose2d()
                            .getTranslation();
                    double fieldRelativeAngle = Math.atan2(
                            ferryGoal2d.getY() - turretTranslation.getY(),
                            ferryGoal2d.getX() - turretTranslation.getX());
                    double turretMountingYaw =
                            RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
                    double turretRelativeAngle = MathUtil.angleModulus(
                            fieldRelativeAngle
                                    - robot.getRotation().getRadians()
                                    - turretMountingYaw);
                    desiredRotation = findBestTurretAngle(
                            turretRelativeAngle, getPosition().getRadians());
                    if (trajectoryLoopCounter % TRAJECTORY_LOG_INTERVAL == 0) {
                        var trajectory = createTrajectory(ferryGoal3d, ferryGoal2d);
                        Logger.recordOutput("Turret/FerryTrajectory",
                                trajectory.toArray(new Translation3d[0]));
                    }
                }
            }

            // ---- Clamp desired rotation to limits ----
            desiredRotation = Rotation2d.fromRadians(
                    MathUtil.clamp(
                            desiredRotation.getRadians(),
                            constants.minRotation().getRadians(),
                            constants.maxRotation().getRadians()));

            // ---- at-goal check — against desiredRotation directly, no profile state ----
            atGoal = Maths.epsilonEquals(
                    getPosition().getRadians(),
                    desiredRotation.getRadians(),
                    tolerance.getAsDouble());

            if (atGoal) {
                io.stop();
            } else {
                // kS and kA now live in Slot0 onboard — only the geometry-based
                // velocity FF (kV * calculateTurretVelocityFF) passes as withFeedForward()
                double geometryFF = kV.getAsDouble()
                        * calculateTurretVelocityFF(velocityTargetFF);

                io.runPivot(desiredRotation.getRadians(), geometryFF);
            }

            Logger.recordOutput("Turret/GoalPositionRad", desiredRotation.getRadians());
        }

        trajectoryLoopCounter++;
        LoopTimeUtil.record("Turret");
    }

    public Rotation2d getRotation() {
        return inputs.positionRad;
    }

    public Command zeroTurret() {
        return Commands.runOnce(() -> {
            isHomed = false;
            isZeroed = false;
        }, this);
    }

    public Command setDesiredRotation(Rotation2d rotation) {
        return Commands.either(
                        Commands.runOnce(() ->
                                desiredRotation = Rotation2d.fromRadians(
                                        MathUtil.clamp(
                                                rotation.getRadians(),
                                                constants.minRotation().getRadians(),
                                                constants.maxRotation().getRadians())), this),
                        Commands.none(),
                        () -> RobotState.getInstance().getAimState().equals(AimState.MANUAL))
                .withName("Set Desired State");
    }

    public Command toggleManualControl(DoubleSupplier joystickAxis) {
        return Commands.either(
                Commands.runOnce(() -> {
                    isManual = false;
                    desiredRotation = getPosition();
                    io.stop();
                }),
                Commands.runOnce(() -> {
                    isManual = true;
                    manualGoalRad = getPosition().getRadians();
                    this.joystickAxis = joystickAxis;
                }),
                () -> isManual);
    }

    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> currentSamples = new LinkedList<>();
        Timer timer = new Timer();
        return Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    currentSamples.clear();
                    timer.restart();
                }),
                Commands.run(() -> {
                            double current = timer.get() * FF_RAMP_RATE;
                            io.runOpenLoop(current, true);
                            velocitySamples.add(inputs.velocityRadPerSec.getRadians());
                            currentSamples.add(current);
                        }, this)
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += currentSamples.get(i);
                                sumXY += velocitySamples.get(i) * currentSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kSResult = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kVResult = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Turret FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kSResult));
                            System.out.println("\tkV: " + formatter.format(kVResult));
                        }));
    }
}