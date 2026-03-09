package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import org.steelhawks.RobotState.ShootingState;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
public class Flywheel extends SubsystemBase {

    private final double[] ampSamples = new double[sampleCounts];
    private double sampledAmps = 0.0;
    private int currentSampleIndex = 0;
    private double timeStartedSampling = 0;
    private final SysIdRoutine routine;
    private final Debouncer setpointDebouncer =
        new Debouncer(0.3, DebounceType.kBoth);

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    public enum FlywheelState {
        RAMP_UP,
        SAMPLING,
        RUNNING
    }

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private FlywheelState state = FlywheelState.RAMP_UP;
    private boolean nearTargetVelocity = false;
    private double targetVelocityRadPerSec = 0.0;

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kV;
    private static LoggedTunableNumber kA;

    private static LoggedTunableNumber velocityTolerance;
    private static LoggedTunableNumber samplingTimeoutDuration;
    private static LoggedTunableNumber timeoutAvgMinSamples;
    public static final int sampleCounts = 50;
    SubsystemConstants.FlywheelConstants constants;

    public Flywheel(FlywheelIO io, SubsystemConstants.FlywheelConstants constants) {
        this.io = io;
        this.constants = constants;
        kP = new LoggedTunableNumber("Flywheel/kP", constants.kP());
        kI = new LoggedTunableNumber("Flywheel/kI", constants.kI());
        kD = new LoggedTunableNumber("Flywheel/kD", constants.kD());
        kS = new LoggedTunableNumber("Flywheel/kS", constants.kS());
        kV = new LoggedTunableNumber("Flywheel/kV", constants.kV());
        kA = new LoggedTunableNumber("Flywheel/kA", constants.kA());
        velocityTolerance
            = new LoggedTunableNumber("Flywheel/VelocityToleranceRadPerSec", constants.velocityToleranceRadPerSec());
        samplingTimeoutDuration =
            new LoggedTunableNumber("Flywheel/SamplingTimeoutDurationSeconds", constants.samplingTimeoutDuration());
        timeoutAvgMinSamples =
            new LoggedTunableNumber("Flywheel/TimeoutMinSamplesForAvgCalculation", constants.samplingTimeoutDuration());
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

        nearTargetVelocity =
            setpointDebouncer.calculate(
                Maths.epsilonEquals(inputs.velocityRadPerSec, targetVelocityRadPerSec, velocityTolerance.get()));
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
                Logger.recordOutput("Flywheel/AimState", RobotState.getInstance().getAimState().name());
                switch (RobotState.getInstance().getAimState()) {
                    case NOTHING -> {
                        double mps = ShooterStructure.Static.calculateShot(
                            FieldConstants.Hub.HUB_CENTER_3D, FieldConstants.Hub.HUB_CENTER_3D, Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
                        double rps = ShooterStructure.linearToAngularVelocity(mps, constants.flywheelRadius());
                        if (rps != targetVelocityRadPerSec) {
                            setTargetVelocity(rps * constants.idleMultiplier());
                        }
                    }
                    case SHOOTING_MOVING -> {
                        double mps = ShooterStructure.Moving.calculateMovingShot(
                            FieldConstants.Hub.HUB_CENTER_3D, Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
                        double rps = ShooterStructure.linearToAngularVelocity(mps, constants.flywheelRadius());
                        if (rps != targetVelocityRadPerSec) {
                            setTargetVelocity(rps);
                        }
                    }
                    case SHOOTING_STATIONARY -> {
                        double mps = ShooterStructure.Static.calculateShot(
                            FieldConstants.Hub.HUB_CENTER_3D, FieldConstants.Hub.HUB_CENTER_3D, Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
                        double rps = ShooterStructure.linearToAngularVelocity(mps, constants.flywheelRadius());
                        if (rps != targetVelocityRadPerSec) {
                            setTargetVelocity(constants.stationaryHoodVelocityFactor() * rps);
                        }
                    }
                }
            }
            double feedforward = ((sampledAmps != 0.0) && Toggles.Flywheel.toggleAdaptiveFeedforward.get())
                ? sampledAmps
                : kS.get() + kV.get() * targetVelocityRadPerSec;
            switch (state) {
                case RAMP_UP -> {
                    io.runProfiledFlywheel(targetVelocityRadPerSec, feedforward, true);
                    if (nearTargetVelocity) {
                        state = FlywheelState.SAMPLING;
                        currentSampleIndex = 0;
                        timeStartedSampling = Timer.getFPGATimestamp();
                    }
                }
                case SAMPLING -> {
                    io.runProfiledFlywheel(targetVelocityRadPerSec, feedforward, true);
                    if (nearTargetVelocity && currentSampleIndex < sampleCounts) {
                        ampSamples[currentSampleIndex] = inputs.torqueCurrentAmps;
                        currentSampleIndex++;
                    }
                    if (currentSampleIndex >= sampleCounts) {
                        sampledAmps = calculateAverageSample();
                        state = FlywheelState.RUNNING;
                        Logger.recordOutput("Flywheel/SampledCurrent", sampledAmps);
                    } else if (Timer.getFPGATimestamp() - timeStartedSampling > samplingTimeoutDuration.get()) {
                        if (currentSampleIndex >= timeoutAvgMinSamples.get()) { // not good not terrible, calculate an average
                            sampledAmps = calculateAverageSample();
                        } else {
                            sampledAmps = 0.0; // calculate the FF voltage using the equation above instead of using the sampled value
                        }

                        state = FlywheelState.RUNNING;
                        Logger.recordOutput("Flywheel/SampledCurrent", sampledAmps);
                    }
                }
                case RUNNING -> {
                    if (nearTargetVelocity) {
                        io.runFlywheelOpenLoop(feedforward, true);
                    } else {
                        // recover velocity if needed
                        io.runProfiledFlywheel(targetVelocityRadPerSec, feedforward, true);
                    }
                }
            }
        } else {
            state = FlywheelState.RAMP_UP;
            sampledAmps = 0.0;
            currentSampleIndex = 0;
            Logger.recordOutput("Flywheel/Feedforward", 0.0);
        }
        Logger.recordOutput("Flywheel/State", state.toString());
        Logger.recordOutput("Flywheel/TargetVelocity", targetVelocityRadPerSec);
        Logger.recordOutput("Flywheel/AdaptiveFeedforward", sampledAmps != 0.0);
    }

    private double calculateAverageSample() {
        if (currentSampleIndex == 0) return 0.0;
        double sum = 0.0;
        for (int i = 0; i < currentSampleIndex; i++) {
            sum += ampSamples[i];
        }
        return sum / currentSampleIndex;
    }

    @AutoLogOutput(key = "Flywheel/ReadyToShoot")
    public boolean isReadyToShoot() {
        return nearTargetVelocity;
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public void setTargetVelocity(double velocityRadPerSec) {
        if (Toggles.shooterTuningMode.get()) return;
        sampledAmps = 0.0;
        targetVelocityRadPerSec = velocityRadPerSec;
        state = FlywheelState.RAMP_UP;
    }

    public void setTargetVelocityForced(double velocityRadPerSec) {
        System.out.println("Working");
        sampledAmps = 0.0;
        targetVelocityRadPerSec = velocityRadPerSec;
        state = FlywheelState.RAMP_UP;
    }

    public Command testfire() {
        return Commands.defer(() ->
            Commands.runOnce(
            () -> {
                var t = ShooterStructure.Moving.calculateMovingShot(FieldConstants.Hub.HUB_CENTER_3D, false);

                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                    // Specify the position of the chassis when the note is launched
                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                    // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
                    Constants.RobotConstants.ROBOT_TO_TURRET.inverse().getTranslation().toTranslation2d(),
                    // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
                    RobotContainer.s_Swerve.getChassisSpeeds(),
                    // The shooter facing direction is the same as the robot’s facing direction
                    RobotContainer.s_Turret.getRotation().plus(Rotation2d.kPi),
                    // Initial height of the flying note
                    Meters.of(Constants.RobotConstants.ROBOT_TO_TURRET.getZ()),
                    // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
                    MetersPerSecond.of(t.exitVelocity()),
                    // The angle at which the note is launched
                    Radians.of(t.hoodAngle())
                );
                fuelOnFly
                    // Configure callbacks to visualize the flight trajectory of the projectile
                    .withProjectileTrajectoryDisplayCallBack(
                        // Callback for when the fuel will eventually hit the target (if configured)
                        (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                        // Callback for when the fuel will eventually miss the target, or if no target is configured
                        (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
                    );
                // Add the projectile to the simulated arena
                SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
            }
        ), Set.of(this));
    }

    public Command shooting() {
        return Commands.run(() -> RobotState.getInstance().setAimState(ShootingState.SHOOTING))
            .finallyDo(() -> RobotState.getInstance().setAimState(ShootingState.NOTHING));
    }

    public Command setTargetVelocityCmd(double velocityRadPerSec) {
        return Commands.runOnce(() -> setTargetVelocity(velocityRadPerSec), this)
        .finallyDo(io::stop);
    }

    public Command sysIdQuasistaic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> Toggles.Flywheel.isEnabled.set(false)).andThen(routine.quasistatic(direction))
            .onlyIf(Toggles.tuningMode::get);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(() -> Toggles.Flywheel.isEnabled.set(false)).andThen(routine.dynamic(direction))
            .onlyIf(Toggles.tuningMode::get);
    }
}
