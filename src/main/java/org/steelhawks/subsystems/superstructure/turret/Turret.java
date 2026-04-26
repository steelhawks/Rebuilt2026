package org.steelhawks.subsystems.superstructure.turret;

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
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.util.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
    private final static double FF_RAMP_RATE = 5.0; // 5 AMPS per sec

    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;

    private static LoggedTunableNumber maxVelocityRadPerSec;
    private static LoggedTunableNumber maxAccelerationRadPerSecSq;
    private static LoggedTunableNumber maxJerkRadPerSecCubed;
    private static LoggedTunableNumber manualIncrement;
    public static double tolerance = Units.degreesToRadians(5.0);
    private static final double SPRING_HYSTERESIS = Units.degreesToRadians(3.0);


    private static LoggedTunableNumber currentHomingThres;
    private static final double homingVolts = 0.1;

    private final Debouncer homingDebouncer = new Debouncer(0.15, DebounceType.kRising);
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final Supplier<Pose2d> poseSupplier;
    private final TurretIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private int trajectoryLoopCounter = 0;
    private static final int TRAJECTORY_LOG_INTERVAL = 5; // log trajectory every 5 loops ~10Hz

    private double manualGoalRad = 0.0;
    private Rotation2d desiredRotation = new Rotation2d();

    private boolean brakeModeEnabled = false;
    private boolean shouldRun = false;
    private boolean isManual = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;
    private DoubleSupplier joystickAxis = null;
    private SubsystemConstants.TurretConstants constants;
    private final LoggedTunableNumber constantForceFF =
        new LoggedTunableNumber("Turret/ConstantForceFF", 45.0);

    private static final double JAM_VELOCITY_THRESHOLD = Units.degreesToRadians(2.0); // rad/s
    private static final double JAM_ERROR_THRESHOLD = Units.degreesToRadians(5.0);
    private static final double JAM_DETECTION_TIME = 0.3; // seconds
    private final Debouncer jamDebouncer = new Debouncer(JAM_DETECTION_TIME, DebounceType.kRising);
    private final Debouncer unjamDebouncer = new Debouncer(0.15, DebounceType.kFalling);

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, SubsystemConstants.TurretConstants constants) {
        this.poseSupplier = poseSupplier;
        this.io = io;
        this.constants = constants;
        kP = new LoggedTunableNumber("Turret/kP", constants.kP());
        kI = new LoggedTunableNumber("Turret/kI", constants.kI());
        kD = new LoggedTunableNumber("Turret/kD", constants.kD());
        kS = new LoggedTunableNumber("Turret/kS", constants.kS());
        kV = new LoggedTunableNumber("Turret/kV", 1.92354 * 1.1);
        kA = new LoggedTunableNumber("Turret/kA", constants.kA());
        maxVelocityRadPerSec =
            new LoggedTunableNumber("Turret/MaxVelocityRadPerSec", constants.maxVelocityRadPerSec());
        maxAccelerationRadPerSecSq =
            new LoggedTunableNumber("Turret/MaxAccelerationRadPerSecSq", constants.maxAccelerationRadPerSecSq());
        maxJerkRadPerSecCubed =
            new LoggedTunableNumber("Turret/MaxJerkRadPerSecCubed", 0.0);
        manualIncrement = new LoggedTunableNumber("Turret/ManualIncrement", constants.manualIncrement());
        currentHomingThres = new LoggedTunableNumber("Turret/CurrentHomingThresholdAmps", constants.currentHomingThreshold());
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

    @AutoLogOutput(key = "Turret/IsJammedOrDeadSpot")
    public boolean isJammedOrInDeadSpot() {
        boolean stuckWithError =
            Math.abs(inputs.velocityRadPerSec.getRadians()) < JAM_VELOCITY_THRESHOLD
                && Math.abs(getPosition().getRadians() - desiredRotation.getRadians()) > JAM_ERROR_THRESHOLD
                && shouldRun;
        boolean jammed = unjamDebouncer.calculate(jamDebouncer.calculate(stuckWithError));
        return jammed || isAtDeadSpot();
    }

    @AutoLogOutput(key = "Turret/IsAtDeadSpot")
    private boolean isAtDeadSpot() {
        if (!shouldRun) return false;
        double goalRad = desiredRotation.getRadians();
        boolean atMinLimit = Math.abs(goalRad - constants.minRotation().getRadians()) < tolerance;
        boolean atMaxLimit = Math.abs(goalRad - constants.maxRotation().getRadians()) < tolerance;
        return (atMinLimit || atMaxLimit) && !atGoal;
    }

    @AutoLogOutput(key = "Turret/IsTraversing")
    public boolean isTraversing() {
        return Math.abs(inputs.positionRad.getRadians() - desiredRotation.getRadians()) > tolerance
            || Math.abs(inputs.velocityRadPerSec.getRadians()) > Units.degreesToRadians(2.0);
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
        // target in blind spot
        if (smallestMove == Double.POSITIVE_INFINITY) {
            bestAngle = MathUtil.clamp(targetAngle,
                constants.minRotation().getRadians(),
                constants.maxRotation().getRadians());
        }
        return Rotation2d.fromRadians(bestAngle);
    }

    private double calculateTurretVelocityFF(Translation2d target2d) {
        // ((v x r_hat_perpendicular) / |r|) - robot_omega
        if (target2d == null) {
            return 0.0;
        }
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
        // velocity of shooter = linear velocity + (omega * r_offset)
        Translation2d robotToTurret = turretPos.minus(robot.getTranslation());
        double turretVx = chassisSpeeds.vxMetersPerSecond - omegaRobot * robotToTurret.getY();
        double turretVy = chassisSpeeds.vyMetersPerSecond + omegaRobot * robotToTurret.getX();
        Translation2d turretVelocity = new Translation2d(turretVx, turretVy);

        Translation2d mrR = target2d.minus(turretPos); // r vector from turret to target
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
//        double timeOfFlight = ShooterStructure.calculateTimeOfFlight(
//            projectileData.exitVelocity(),
//            launchAngle,
//            turretTranslation.getDistance(target2d)
//        );
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
            // to field relative
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

    public void freezeAtCurrentPosition() {
        desiredRotation = getPosition();
    }

    public Rotation2d getMinRotation() {
        return constants.minRotation();
    }

    public Rotation2d getMaxRotation() {
        return constants.minRotation();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        BatteryUtil.recordCurrentUsage("Turret", inputs.supplyCurrentAmps);
        if (Constants.getRobot().equals(RobotType.SIMBOT) && !isHomed && !isZeroed) {
            isHomed = true;
            Logger.recordOutput("Turret/IsHomed", true);
            io.setPosition(0);
            isZeroed = true;
            // sync setpoint to new position immediately, so turret doesnt violently snap like we've been seeing
            desiredRotation = Rotation2d.fromRadians(Math.PI);
            Logger.recordOutput("Turret/Zeroed", true);
        }
        if (!isHomed && Toggles.Turret.isEnabled.get()) {
            io.runPercentOutput(homingVolts);
            isHomed = homingDebouncer.calculate(inputs.supplyCurrentAmps > currentHomingThres.getAsDouble());
            Logger.recordOutput("Turret/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(-Math.PI / 2);
                io.stop();
                isZeroed = true;
                // sync setpoint to new position immediately, so turret doesnt violently snap like we've been seeing
                desiredRotation = Rotation2d.fromRadians(Math.PI);
                Logger.recordOutput("Turret/Zeroed", true);
            }
        }
        shouldRun =
            DriverStation.isEnabled()
                && !isManual
                && ((isHomed && isZeroed) || Constants.getRobot().equals(RobotType.SIMBOT))
                && Toggles.Turret.isEnabled.get()
                && (inputs.connected && (inputs.encoderConnected || Constants.getRobot().equals(RobotType.ALPHABOT))) // add check to make sure it only checks for omega
                && !Toggles.Turret.toggleVoltageOverride.get()
                && !Toggles.Turret.toggleCurrentOverride.get()
                && (getPosition().getRadians() <= constants.maxRotation().getRadians()
                    && getPosition().getRadians() >= constants.minRotation().getRadians());
        Logger.recordOutput("Turret/ShouldRun", shouldRun);
        if (DriverStation.isDisabled()) {
            desiredRotation = getPosition();
        }
        if (DriverStation.isDisabled() && Robot.isFirstRun()) {
            setBrakeMode(false);
        }
        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }
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
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setPID(
                    kP.get(),
                    kI.get(),
                    kD.get());
            }, kP, kI, kD);
            if (maxVelocityRadPerSec.hasChanged(hashCode())
                || maxAccelerationRadPerSecSq.hasChanged(hashCode())
            ) {
                io.setMotionMagic(
                    maxVelocityRadPerSec.get(),
                    maxAccelerationRadPerSecSq.get(),
                    maxJerkRadPerSecCubed.get());
            }
        }
        if (isManual && joystickAxis != null) {
            double delta =
                joystickAxis.getAsDouble()
                    * manualIncrement.getAsDouble();
            manualGoalRad =
                MathUtil.clamp(
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
            switch (RobotState.getInstance().getAimState()) {
                case TO_HUB -> {
                    var robot = getPose();
                    var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
                    var sol = RobotState.getInstance().getMovingShotSolution();
                    if (sol != null && RobotState.getInstance().getShootingState().equals(ShootingState.SHOOTING_MOVING)) {
                        velocityTargetFF = sol.virtualTarget().toTranslation2d();
                        desiredRotation = findBestTurretAngle(
                            sol.turretAngle().getRadians(),
                            getPosition().getRadians());
                        if (trajectoryLoopCounter % TRAJECTORY_LOG_INTERVAL == 0) {
                            var trajectory = createTrajectory(hubCenter, sol.virtualTarget().toTranslation2d());
                            Logger.recordOutput("Turret/ScoreTrajectory", trajectory.toArray(new Translation3d[0]));
                        }
                    } else {
                        // fallback aim directly at hub with no velocity compensation
                        velocityTargetFF = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER);
                        var turretTranslation = new Pose3d(robot)
                            .transformBy(RobotConstants.ROBOT_TO_TURRET)
                            .toPose2d()
                            .getTranslation();
                        double fieldRelativeAngle = Math.atan2(
                            hubCenter.getY() - turretTranslation.getY(),
                            hubCenter.getX() - turretTranslation.getX());
                        double turretMountingYaw = RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
                        double turretRelativeAngle = MathUtil.angleModulus(
                            fieldRelativeAngle - robot.getRotation().getRadians() - turretMountingYaw);
                        desiredRotation = findBestTurretAngle(turretRelativeAngle, getPosition().getRadians());
                        if (trajectoryLoopCounter % TRAJECTORY_LOG_INTERVAL == 0) {
                            var trajectory = createTrajectory(hubCenter, hubCenter.toTranslation2d());
                            Logger.recordOutput("Turret/ScoreTrajectory", trajectory.toArray(new Translation3d[0]));
                        }
                    }
                }
                case FERRY -> {
                    var robot = getPose();
                    var ferryGoal2d = ShooterStructure.Static.calculateFerryShotSetpoint();
                    var ferryGoal3d = new Translation3d(ferryGoal2d.getX(), ferryGoal2d.getY(), 0.0);
                    var ferrySol = RobotState.getInstance().getMovingShotSolution();
                    if (ferrySol != null && RobotState.getInstance().getShootingState().equals(ShootingState.SHOOTING_MOVING)) {
                        desiredRotation = findBestTurretAngle(
                            ferrySol.turretAngle().getRadians(),
                            getPosition().getRadians());
                        if (trajectoryLoopCounter % TRAJECTORY_LOG_INTERVAL == 0) {
                            var trajectory = createTrajectory(ferryGoal3d, ferrySol.virtualTarget().toTranslation2d());
                            Logger.recordOutput("Turret/FerryTrajectory", trajectory.toArray(new Translation3d[0]));
                        }
                    } else {
                        var turretTranslation = new Pose3d(robot)
                            .transformBy(RobotConstants.ROBOT_TO_TURRET)
                            .toPose2d()
                            .getTranslation();
                        double fieldRelativeAngle = Math.atan2(
                            ferryGoal2d.getY() - turretTranslation.getY(),
                            ferryGoal2d.getX() - turretTranslation.getX());
                        double turretMountingYaw = RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
                        double turretRelativeAngle = MathUtil.angleModulus(
                            fieldRelativeAngle - robot.getRotation().getRadians() - turretMountingYaw);
                        desiredRotation = findBestTurretAngle(turretRelativeAngle, getPosition().getRadians());
                        if (trajectoryLoopCounter % TRAJECTORY_LOG_INTERVAL == 0) {
                            var trajectory = createTrajectory(ferryGoal3d, ferryGoal2d);
                            Logger.recordOutput("Turret/FerryTrajectory", trajectory.toArray(new Translation3d[0]));
                        }
                    }
                }
            }
            desiredRotation =
                Rotation2d.fromRadians(
                    MathUtil.clamp(
                        desiredRotation.getRadians(), constants.minRotation().getRadians(), constants.maxRotation().getRadians()));
            atGoal = Maths.epsilonEquals(getPosition().getRadians(), desiredRotation.getRadians(), tolerance);

            boolean springPullsNegative = getPosition().getRadians() <= -0.984816 + SPRING_HYSTERESIS;
            boolean springPullsPositive = getPosition().getRadians() >= 1.810097 - SPRING_HYSTERESIS;

            double constantForceSpringFF = 0.0;
            double pos = getPosition().getRadians();
            double negThreshold = -0.984816 + SPRING_HYSTERESIS;
            double posThreshold = 1.810097 - SPRING_HYSTERESIS;

            if (pos <= negThreshold) {
                double depth = (negThreshold - pos) / (negThreshold - constants.minRotation().getRadians());
                constantForceSpringFF = constantForceFF.getAsDouble() * Math.min(depth, 1.0);
            } else if (pos >= posThreshold) {
                double depth = (pos - posThreshold) / (constants.maxRotation().getRadians() - posThreshold);
                constantForceSpringFF = -constantForceFF.getAsDouble() * Math.min(depth, 1.0);
            }
            Logger.recordOutput("Turret/ForceSpringActive", springPullsNegative || springPullsPositive);
            io.runPivotMM(
                desiredRotation.getRadians(),
                kV.getAsDouble() * calculateTurretVelocityFF(velocityTargetFF)
                    + constantForceSpringFF
            );
            Logger.recordOutput("Turret/GoalPosition", desiredRotation.getRadians());
        } else {
            desiredRotation = new Rotation2d(0.0);
            Logger.recordOutput("Turret/SetpointPosition", 0.0);
            Logger.recordOutput("Turret/SetpointVelocity", 0.0);
            Logger.recordOutput("Turret/GoalPosition", 0.0);
            Logger.recordOutput("Turret/GoalVelocity", 0.0);
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
        return Commands.runOnce(() -> RobotState.getInstance().setAimState(AimState.MANUAL)).andThen(
            Commands.either(
                Commands.runOnce(
                    () -> desiredRotation =
                            Rotation2d.fromRadians(
                                MathUtil.clamp(
                                    rotation.getRadians(), constants.minRotation().getRadians(), constants.maxRotation().getRadians())), this),
                    Commands.none(),
                    () -> RobotState.getInstance().getAimState().equals(AimState.MANUAL)))
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
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    currentSamples.clear();
                    timer.restart();
                }),
            // Accelerate and gather data
            Commands.run(
                    () -> {
                        double current = timer.get() * FF_RAMP_RATE;
                        io.runOpenLoop(current, true);
                        velocitySamples.add(inputs.velocityRadPerSec.getRadians());
                        currentSamples.add(current);
                    },
                    this)
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += currentSamples.get(i);
                            sumXY += velocitySamples.get(i) * currentSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                        NumberFormat formatter = new DecimalFormat("#0.00000");
                        System.out.println("********** Turret FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                    }));
    }
}
