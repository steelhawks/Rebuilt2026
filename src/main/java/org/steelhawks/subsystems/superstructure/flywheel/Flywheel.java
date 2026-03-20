package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.steelhawks.util.AllianceFlip;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

import java.util.Set;

import static edu.wpi.first.units.Units.*;

public class Flywheel extends SubsystemBase {

    private final SysIdRoutine routine;
    private final Debouncer setpointDebouncer =
        new Debouncer(0.3, DebounceType.kBoth);

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private boolean nearTargetVelocity = false;
    private double targetVelocityRadPerSec = 0.0;

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kV;

    private static double stationaryHoodVelocityFactor;

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
        stationaryHoodVelocityFactor = constants.stationaryHoodVelocityFactor();
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
                var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
                switch (RobotState.getInstance().getAimState()) {
                    case NOTHING -> {
                        double mps = ShooterStructure.Static.calculateShot(
                            hubCenter, hubCenter,
                            Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
                        double rps = ShooterStructure.linearToAngularVelocity(mps, constants.flywheelRadius());
                        if (Math.abs(rps - targetVelocityRadPerSec) > 0.5) {
                            setTargetVelocity(rps * constants.idleMultiplier());
                        }
                    }
                    case SHOOTING_MOVING -> {
                        double mps = ShooterStructure.Moving.calculateMovingShot(
                            hubCenter,
                            Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
                        double rps = ShooterStructure.linearToAngularVelocity(stationaryHoodVelocityFactor * mps, constants.flywheelRadius());
                        setTargetVelocity(rps);
                    }
                    case SHOOTING_STATIONARY -> {
                        double mps = ShooterStructure.Static.calculateShot(
                            hubCenter, hubCenter,
                            Constants.getRobot().equals(RobotType.ALPHABOT)).exitVelocity();
                        double rps = ShooterStructure.linearToAngularVelocity(
                            stationaryHoodVelocityFactor * mps, constants.flywheelRadius());
                        setTargetVelocity(rps);
                    }
                }
            }

            double feedforward = kS.get() + kV.get() * targetVelocityRadPerSec;
            io.runFlywheel(targetVelocityRadPerSec, feedforward, false);
        }

        Logger.recordOutput("Flywheel/TargetVelocity", targetVelocityRadPerSec);
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
        if (Double.isNaN(velocityRadPerSec) || Double.isInfinite(velocityRadPerSec)) {
            Logger.recordOutput("Flywheel/InvalidSetpointRejected", true);
            return;
        }
        targetVelocityRadPerSec = velocityRadPerSec;
    }

    public void setTargetVelocityForced(double velocityRadPerSec) {
        targetVelocityRadPerSec = velocityRadPerSec;
    }

    public Command testfire() {
        return Commands.defer(() ->
            Commands.runOnce(
                () -> {
                    var t = ShooterStructure.Moving.calculateMovingShot(FieldConstants.Hub.HUB_CENTER_3D, false);

                    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                        RobotState.getInstance().getEstimatedPose().getTranslation(),
                        Constants.RobotConstants.ROBOT_TO_TURRET.inverse().getTranslation().toTranslation2d(),
                        RobotContainer.s_Swerve.getChassisSpeeds(),
                        RobotContainer.s_Turret.getRotation().plus(Rotation2d.kPi),
                        Meters.of(Constants.RobotConstants.ROBOT_TO_TURRET.getZ()),
                        MetersPerSecond.of(t.exitVelocity()),
                        Radians.of(t.hoodAngle())
                    );
                    fuelOnFly
                        .withProjectileTrajectoryDisplayCallBack(
                            (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                            (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
                        );
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

    public Command setTargetVelocityForcedCmd(double velocityRadPerSec) {
        return Commands.runOnce(() -> setTargetVelocityForced(velocityRadPerSec), this)
            .finallyDo(io::stop);
    }

    public Command incrementVelocityFactor(double increment) {
        return Commands.runOnce(() -> {
            stationaryHoodVelocityFactor += increment;
            Logger.recordOutput("Flywheel/VelocityFactor", stationaryHoodVelocityFactor);
        });
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