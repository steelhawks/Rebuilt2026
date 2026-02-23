package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.Constants.RobotConstants;
import org.steelhawks.RobotState.ShooterMode;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.LoopTimeUtil;
import org.steelhawks.util.Maths;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {

    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", Constants.omega(2.0, 0.2));
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", Constants.omega(0.0, 0.0));
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", Constants.omega(1000.0, 200.0)); // 1500
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", Constants.omega(0.0, 0.0));
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", Constants.omega(70.0, 7.0)); // 35

    private static final LoggedTunableNumber maxVelocityRadPerSec = new LoggedTunableNumber("Turret/MaxVelocityRadPerSec", 10.0);
    private static final LoggedTunableNumber maxAccelerationRadPerSecSq = new LoggedTunableNumber("Turret/MaxAccelerationRadPerSecSq", 5.0);
    private static final LoggedTunableNumber tolerance = new LoggedTunableNumber("Turret/Tolerance", Math.PI / 60.0); // 3deg
    private static final LoggedTunableNumber manualIncrement = new LoggedTunableNumber("Turret/ManualIncrement", 0.1);

    private static final LoggedTunableNumber currentHomingThres =
        new LoggedTunableNumber("Turret/CurrentHomingThreshold", 40.0);
    private static final double homingVolts = 0.1;

    private static final Rotation2d minRotation = new Rotation2d(Constants.value((-Math.PI / 2.0), 0.0) - (Math.PI / 60.0));
    private static final Rotation2d maxRotation = new Rotation2d(Constants.value(Math.PI, 2 * Math.PI) + (Math.PI / 60.0));
    public static int motorId = 1;

    private final Debouncer homingDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private Supplier<Pose2d> poseSupplier;
    private TrapezoidProfile profile;
    private final TurretIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private double manualGoalRad = 0.0;
    private Rotation2d desiredRotation = new Rotation2d();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private boolean brakeModeEnabled = false;
    private boolean isManual = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;
    private DoubleSupplier joystickAxis = null;

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.io = io;
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    maxVelocityRadPerSec.getAsDouble(),
                    maxAccelerationRadPerSecSq.getAsDouble()));
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
            if (candidate >= minRotation.getRadians() &&
                candidate <= maxRotation.getRadians()) {
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
                minRotation.getRadians(),
                maxRotation.getRadians());
        }
        return Rotation2d.fromRadians(bestAngle);
    }

    private Translation3d predictInterceptPoint(Translation3d actualTarget) {
        Translation3d robotVelocity = new Translation3d(
            RobotContainer.s_Swerve.getChassisSpeeds().vxMetersPerSecond,
            RobotContainer.s_Swerve.getChassisSpeeds().vyMetersPerSecond,
            0.0);
        Translation3d predictedTarget = actualTarget;
        int maxIterations = 5;
        double convergenceThreshold = 0.01;
        for (int i = 0; i < maxIterations; i++) {
            var turretTranslation = new Pose3d(getPose())
                .transformBy(RobotConstants.ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation();
            ShooterStructure.ProjectileData solution = ShooterStructure.Static.calculateShot(actualTarget, predictedTarget);
            if (solution == null || Double.isNaN(solution.exitVelocity())) break;
            double distance = turretTranslation.getDistance(predictedTarget.toTranslation2d());
            double tof = ShooterStructure.calculateTimeofFlight(
                solution.exitVelocity(),
                solution.hoodAngle(),
                distance);
            Translation3d newPredictedTarget = actualTarget.minus(robotVelocity.times(tof));
            double error = predictedTarget.getDistance(newPredictedTarget);
            Logger.recordOutput("Turret/Moving/IterationError_" + i, error);
            if (error < convergenceThreshold) break;

            predictedTarget = newPredictedTarget;
        }
        Logger.recordOutput("Turret/Moving/PredictedTarget",
            new double[]{predictedTarget.getX(), predictedTarget.getY(), predictedTarget.getZ()});
        return predictedTarget;
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
        if (projectileData == null) {
            return new ArrayList<>();
        }
        double launchAngle = projectileData.hoodAngle();
        double timeOfFlight = ShooterStructure.calculateTimeofFlight(
            projectileData.exitVelocity(),
            launchAngle,
            turretTranslation.getDistance(target2d)
        );

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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        if (Constants.getRobot().equals(Constants.RobotType.SIMBOT) && !isHomed && !isZeroed) {
            isHomed = true;
            Logger.recordOutput("Turret/IsHomed", true);
            io.setPosition(0);
            isZeroed = true;
            Logger.recordOutput("Turret/Zeroed", true);
        }
        if (!isHomed && Toggles.Turret.isEnabled.get()) {
            io.runPercentOutput(homingVolts);
            isHomed = homingDebouncer.calculate(inputs.currentAmps > currentHomingThres.getAsDouble());
            Logger.recordOutput("Turret/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(Math.PI);
                io.stop();
                isZeroed = true;
                Logger.recordOutput("Turret/Zeroed", true);
            }
        }
        final boolean shouldRun =
            DriverStation.isEnabled()
                && !isManual
                && ((isHomed && isZeroed) || Constants.getRobot().equals(Constants.RobotType.SIMBOT))
                && inputs.connected
                && Toggles.Turret.isEnabled.get()
                && !Toggles.Turret.toggleVoltageOverride.get()
                && !Toggles.Turret.toggleCurrentOverride.get()
                && (getPosition().getRadians() <= maxRotation.getRadians()
                    && getPosition().getRadians() >= minRotation.getRadians());
        Logger.recordOutput("Turret/ShouldRun", shouldRun);
        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(getPosition().getRadians(), 0.0);
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
                profile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            maxVelocityRadPerSec.get(),
                            maxAccelerationRadPerSecSq.get()));
            }
        }
        if (isManual && joystickAxis != null) {
            double delta =
                joystickAxis.getAsDouble()
                    * manualIncrement.getAsDouble();
            manualGoalRad =
                MathUtil.clamp(
                    manualGoalRad + delta,
                    minRotation.getRadians(),
                    maxRotation.getRadians());
            desiredRotation = Rotation2d.fromRadians(manualGoalRad);
        }
        if (shouldRun) {
            switch (RobotState.getInstance().getShooterMode()) {
                case TO_HUB -> {
                    var robot = getPose();
                    var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
                    var predictedHub = predictInterceptPoint(hubCenter);
                    var turretTranslation = new Pose3d(robot)
                        .transformBy(RobotConstants.ROBOT_TO_TURRET)
                        .toPose2d()
                        .getTranslation();
                    var direction = predictedHub.toTranslation2d().minus(turretTranslation);
                    double fieldRelativeAngle = direction.getAngle().getRadians();
                    double turretMountingYaw = RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
                    double turretRelativeAngle = MathUtil.angleModulus(
                        fieldRelativeAngle - robot.getRotation().getRadians() - turretMountingYaw);
                    desiredRotation = findBestTurretAngle(turretRelativeAngle, getPosition().getRadians());
                    var trajectory = createTrajectory(hubCenter, predictedHub.toTranslation2d());
                    Logger.recordOutput("Turret/ScoreTrajectory", trajectory.toArray(new Translation3d[0]));
                }
                case FERRY -> {
                    var robot = getPose();
                    var ferryGoal2d = AllianceFlip.apply(
                        FieldConstants.getClosestPointOnLine(FieldConstants.Ferrying.START_LINE, FieldConstants.Ferrying.END_LINE));
                    var ferryGoal3d = new Translation3d(ferryGoal2d.getX(), ferryGoal2d.getY(), 0.0);
                    var predictedFerry = predictInterceptPoint(ferryGoal3d);
                    var turretTranslation = new Pose3d(robot)
                        .transformBy(RobotConstants.ROBOT_TO_TURRET)
                        .toPose2d()
                        .getTranslation();
                    var direction = predictedFerry.toTranslation2d().minus(turretTranslation);
                    double fieldRelativeAngle = direction.getAngle().getRadians();
                    double turretMountingYaw = RobotConstants.ROBOT_TO_TURRET.getRotation().getZ();
                    double turretRelativeAngle = MathUtil.angleModulus(
                        fieldRelativeAngle - robot.getRotation().getRadians() - turretMountingYaw);
                    desiredRotation = findBestTurretAngle(turretRelativeAngle, getPosition().getRadians());
                    var trajectory = createTrajectory(ferryGoal3d, predictedFerry.toTranslation2d());
                    Logger.recordOutput("Turret/FerryTrajectory", trajectory.toArray(new Translation3d[0]));
                }
            }
            desiredRotation =
                Rotation2d.fromRadians(
                    MathUtil.clamp(
                        desiredRotation.getRadians(), minRotation.getRadians(), maxRotation.getRadians()));
            goal = new TrapezoidProfile.State(desiredRotation.getRadians(), 0.0);
            double previousVelocity = setpoint.velocity;
            setpoint =
                profile
                    .calculate(Constants.UPDATE_LOOP_DT, setpoint, goal);
            if (setpoint.position < minRotation.getRadians()
                || setpoint.position > maxRotation.getRadians()
            ) {
                setpoint =
                    new TrapezoidProfile.State(
                        MathUtil.clamp(setpoint.position, minRotation.getRadians(), maxRotation.getRadians()),
                        0.0);
            }
            atGoal = Maths.epsilonEquals(getPosition().getRadians(), goal.position, tolerance.getAsDouble());
            if (atGoal) {
                io.stop();
            } else {
                double acceleration = (setpoint.velocity - previousVelocity) / Constants.UPDATE_LOOP_DT;
                io.runPivot(
                    setpoint.position,
                    kS.getAsDouble() * Math.signum(setpoint.velocity)
                        + kA.getAsDouble() * acceleration
                );
            }
            Logger.recordOutput("Turret/SetpointPosition", setpoint.position);
            Logger.recordOutput("Turret/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Turret/GoalPosition", goal.position);
            Logger.recordOutput("Turret/GoalVelocity", goal.velocity);
        } else {
            setpoint = new TrapezoidProfile.State(getPosition().getRadians(), 0.0);
            Logger.recordOutput("Turret/SetpointPosition", 0.0);
            Logger.recordOutput("Turret/SetpointVelocity", 0.0);
            Logger.recordOutput("Turret/GoalPosition", 0.0);
            Logger.recordOutput("Turret/GoalVelocity", 0.0);
        }
        LoopTimeUtil.record("Turret");
    }

    public Command setDesiredRotation(Rotation2d rotation) {
        return Commands.either(
            Commands.runOnce(
                () -> desiredRotation =
                        Rotation2d.fromRadians(
                            MathUtil.clamp(
                                rotation.getRadians(), minRotation.getRadians(), maxRotation.getRadians())), this),
                Commands.none(),
                () -> RobotState.getInstance().getShooterMode().equals(ShooterMode.MANUAL))
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
}
