package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.Constants.RobotType;
import org.steelhawks.RobotState.AimState;
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.Toggles;
import org.steelhawks.commands.rumble.RumbleAPI;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.BatteryUtil;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import static edu.wpi.first.units.Units.*;

public class Flywheel extends SubsystemBase {

    private final static double FF_RAMP_RATE = 10.0; // 10 AMPS per sec

    private final SysIdRoutine routine;
    private final Debouncer setpointDebouncer =
        new Debouncer(0.6, DebounceType.kBoth);

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private boolean nearTargetVelocity = false;
    private double targetVelocityRadPerSec = 0.0;

    private double cachedStationaryMps = Double.NaN;
    private double cachedStationaryDist = Double.NaN;
    private static final double IDLE_SHOT_CACHE_THRESHOLD = 0.01; // meters

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kV;

    private static double redBullConstant;

    private boolean bumpUpSpeed = false;

    private static LoggedTunableNumber velocityTolerance;
    SubsystemConstants.FlywheelConstants constants;

    public Flywheel(FlywheelIO io, SubsystemConstants.FlywheelConstants constants) {
        this.io = io;
        this.constants = constants;
        kP = new LoggedTunableNumber("Flywheel/kP", constants.kP());
        kI = new LoggedTunableNumber("Flywheel/kI", constants.kI());
        kD = new LoggedTunableNumber("Flywheel/kD", constants.kD());
        kS = new LoggedTunableNumber("Flywheel/kS", constants.kS());
        kV = new LoggedTunableNumber("Flywheel/kV", constants.kV());
        redBullConstant = constants.stationaryHoodVelocityFactor();
        velocityTolerance =
            new LoggedTunableNumber("Flywheel/VelocityToleranceRadPerSec", constants.velocityToleranceRadPerSec());
        routine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> io.runFlywheelOpenLoop(voltage.in(Volts), false), null, this)
            );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        BatteryUtil.recordCurrentUsage("Flywheel", inputs.leftSupplyCurrentAmps + inputs.rightSupplyCurrentAmps);
        Logger.recordOutput("Flywheel/BumpSpeed", bumpUpSpeed);
        redBullConstant = Toggles.useLUT.get() ? ((bumpUpSpeed ? 1.04 : 1.0)) : constants.stationaryHoodVelocityFactor();

        nearTargetVelocity =
            setpointDebouncer.calculate(
                Maths.epsilonEquals((inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0, targetVelocityRadPerSec, velocityTolerance.get()));

        final boolean shouldRun =
            DriverStation.isEnabled()
                && Toggles.Flywheel.isEnabled.get()
                && !Toggles.Flywheel.toggleVoltageOverride.get()
                && !Toggles.Flywheel.toggleCurrentOverride.get();

        if (Toggles.tuningMode.get()) {
            if (Toggles.Flywheel.toggleVoltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Flywheel/TuningVolts", 0.0);
                }
                io.runFlywheelOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Flywheel.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Flywheel/TuningAmps", 0.0);
                }
                io.runFlywheelOpenLoop(tuningAmps.get(), true);
            }
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setPID(kP.get(), kI.get(), kD.get());
            }, kP, kI, kD);
        }
        if (shouldRun) {
            if (!Toggles.shooterTuningMode.get()) {
                Logger.recordOutput("Flywheel/AimState", RobotState.getInstance().getShootingState().name());
                var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
                switch (RobotState.getInstance().getShootingState()) {
                    case NOTHING -> {
                        double mps = getStationaryExitVelocityMps(hubCenter);
                        double rps = ShooterStructure.linearToAngularVelocity(mps, constants.flywheelRadius());
                        if (Math.abs(rps - targetVelocityRadPerSec) > 0.5) {
                            setTargetVelocity(rps * constants.idleMultiplier());
                        }
                    }
                    case SHOOTING_MOVING -> {
                        var sol = RobotState.getInstance().getMovingShotSolution();
                        if (sol != null) {
                            double rps = ShooterStructure.linearToAngularVelocity(
                                redBullConstant * sol.exitVelocity()
                                     * (DriverStation.isAutonomous()
                                        ? 1.06
                                        : 1.0),
                                constants.flywheelRadius());
                            setTargetVelocity(rps);
                        }
                    }
                    case SHOOTING_STATIONARY -> {
                        double mps = getStationaryExitVelocityMps(hubCenter);
                        double rps = ShooterStructure.linearToAngularVelocity(
                            redBullConstant * mps, constants.flywheelRadius());
                        setTargetVelocity(rps);
                    }
                }
            }
            double feedforward = kS.get() + kV.get() * targetVelocityRadPerSec;
            io.runFlywheel(targetVelocityRadPerSec, feedforward, true);
        }
        Logger.recordOutput("Flywheel/TargetVelocity", targetVelocityRadPerSec);
    }

    @AutoLogOutput(key = "Flywheel/ReadyToShoot")
    public boolean isReadyToShoot() {
        return nearTargetVelocity;
    }

    private double getStationaryExitVelocityMps(Translation3d hubCenter) {
        if (!RobotState.getInstance().getAimState().equals(AimState.TO_HUB)) {
            return ShooterStructure.Static.calculateFerryShot(ShooterStructure.Static.calculateFerryShotSetpoint()).exitVelocity();
        }
        double currentDist = ShooterStructure.distanceToTarget(hubCenter);
        if (Double.isNaN(cachedStationaryDist) || Math.abs(currentDist - cachedStationaryDist) > IDLE_SHOT_CACHE_THRESHOLD) {
            cachedStationaryMps = ShooterStructure.Static.calculateShot(hubCenter, hubCenter, Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
            cachedStationaryDist = currentDist;
        }
        return cachedStationaryMps;
    }

    public double getStatorCurrentAmps() {
        return inputs.leftTorqueCurrentAmps;
    }

    public void setTargetVelocity(double velocityRadPerSec) {
        if (Toggles.shooterTuningMode.get()) return;
        if (Double.isNaN(velocityRadPerSec) || Double.isInfinite(velocityRadPerSec)) {
            Logger.recordOutput("Flywheel/InvalidSetpointRejected", true);
            return;
        }
        targetVelocityRadPerSec = velocityRadPerSec;
    }

    public void setTargetVelocityForced(double velocityRadPerSec) {
        targetVelocityRadPerSec = velocityRadPerSec;
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command toggleBumpUp() {
        return Commands.runOnce(
            () -> bumpUpSpeed = !bumpUpSpeed)
            .alongWith(RumbleAPI.steady(1.0, 1.0));
    }

    public Command simFire() {
        return Commands.defer(() ->
            Commands.runOnce(
                () -> {
                    var sol = RobotState.getInstance().getMovingShotSolution();
                    if (sol == null) return;

                    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                        RobotState.getInstance().getEstimatedPose().getTranslation(),
                        Constants.RobotConstants.ROBOT_TO_TURRET.inverse().getTranslation().toTranslation2d(),
                        RobotContainer.s_Swerve.getChassisSpeeds(),
                        RobotContainer.s_Turret.getRotation().plus(Rotation2d.kPi),
                        Meters.of(Constants.RobotConstants.ROBOT_TO_TURRET.getZ()),
                        MetersPerSecond.of(sol.exitVelocity()),
                        Radians.of(sol.hoodAngleRad())
                    );
                    fuelOnFly.withProjectileTrajectoryDisplayCallBack(
                        (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                        (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
                    );
                    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
                }
            ), Set.of(this));
    }

    public Command shooting() {
        return Commands.run(() -> RobotState.getInstance().setShootingState(ShootingState.SHOOTING))
            .finallyDo(() -> RobotState.getInstance().setShootingState(ShootingState.NOTHING));
    }

    public Command setTargetVelocityCmd(double velocityRadPerSec) {
        return Commands.runOnce(() -> setTargetVelocity(velocityRadPerSec), this)
            .finallyDo(io::stop);
    }

    public Command setTargetVelocityForcedCmd(double velocityRadPerSec) {
        return Commands.runOnce(() -> setTargetVelocityForced(velocityRadPerSec), this)
            .finallyDo(io::stop);
    }

    public Command incrementVelocityFactor(double increment) {
        return Commands.runOnce(() -> {
            redBullConstant += increment;
            Logger.recordOutput("Flywheel/VelocityFactor", redBullConstant);
        });
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
                        io.runFlywheelOpenLoop(current, true);
                        velocitySamples.add((inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0);
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
                        System.out.println("********** Flywheel FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                    }));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> Toggles.Flywheel.isEnabled.set(false)).andThen(routine.quasistatic(direction))
            .onlyIf(Toggles.tuningMode::get);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> Toggles.Flywheel.isEnabled.set(false)).andThen(routine.dynamic(direction))
            .onlyIf(Toggles.tuningMode::get);
    }
}