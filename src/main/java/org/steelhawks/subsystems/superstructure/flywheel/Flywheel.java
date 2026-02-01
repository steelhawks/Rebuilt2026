package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.Maths;

import java.util.Set;

import static edu.wpi.first.units.Units.Volts;

public class Flywheel extends SubsystemBase {

    public static final int motorId1 = 2;
    public static final int motorId2 = 3;
    public static final LoggedTunableNumber kP =
        new LoggedTunableNumber("Flywheel/kP", 0.1);
    public static final LoggedTunableNumber kI =
        new LoggedTunableNumber("Flywheel/kI", 0.0);
    public static final LoggedTunableNumber kD =
        new LoggedTunableNumber("Flywheel/kD", 0.0);
    public static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Flywheel/kS", 0.22382);
    public static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Flywheel/kV", 0.0080032);
    public static final LoggedTunableNumber velocityTolerance =
        new LoggedTunableNumber("Flywheel/VelocityToleranceRadPerSec", 5.0);

    private static final int sampleCounts = 50;
    private final double[] voltageSamples = new double[sampleCounts];
    private double sampledVoltage = 0.0;
    private int currentSampleIndex = 0;
    private final SysIdRoutine routine;

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

    public Flywheel(FlywheelIO io) {
        this.io = io;
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

        nearTargetVelocity = Maths.epsilonEquals(inputs.velocityRadPerSec, targetVelocityRadPerSec, velocityTolerance.get());
        final boolean shouldRun =
            DriverStation.isEnabled()
                && Toggles.Flywheel.isEnabled.get()
                && !Toggles.Turret.toggleVoltageOverride.get()
                && !Toggles.Turret.toggleCurrentOverride.get()
                && !Toggles.tuningMode.get();
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
            double feedforward = ((sampledVoltage != 0.0) && Toggles.Flywheel.toggleAdaptiveFeedforward.get())
                ? sampledVoltage
                : kS.get() + kV.get() * targetVelocityRadPerSec;
            switch (state) {
                case RAMP_UP -> {
                    io.runFlywheel(targetVelocityRadPerSec, feedforward, false);
                    if (nearTargetVelocity) {
                        state = FlywheelState.SAMPLING;
                        currentSampleIndex = 0;
                    }
                }
                case SAMPLING -> {
                    io.runFlywheel(targetVelocityRadPerSec, feedforward, false);
                    if (nearTargetVelocity && currentSampleIndex < sampleCounts) {
                        voltageSamples[currentSampleIndex] = inputs.appliedVolts;
                        currentSampleIndex++;
                    }
                    if (currentSampleIndex >= sampleCounts) {
                        sampledVoltage = calculateAverageSample();
                        state = FlywheelState.RUNNING;
                        Logger.recordOutput("Flywheel/SampledVoltage", sampledVoltage);
                    }
                }
                case RUNNING -> io.runFlywheel(targetVelocityRadPerSec, feedforward, false);
            }
        } else {
            state = FlywheelState.RAMP_UP;
            sampledVoltage = 0.0;
            currentSampleIndex = 0;
            Logger.recordOutput("Flywheel/Feedforward", 0.0);
        }
        Logger.recordOutput("Flywheel/State", state.toString());
        Logger.recordOutput("Flywheel/TargetVelocity", targetVelocityRadPerSec);
        Logger.recordOutput("Flywheel/AdaptiveFeedforward", sampledVoltage != 0.0);
    }

    private double calculateAverageSample() {
        double sum = 0.0;
        for (double sample : voltageSamples) {
            sum += sample;
        }
        return sum / sampleCounts;
    }

    public boolean isReadyToShoot() {
        return state == FlywheelState.RUNNING && nearTargetVelocity;
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command setTargetVelocity(double velocityRadPerSec) {
        return Commands.defer(() -> Commands.runOnce(() -> {
            sampledVoltage = 0.0;
            targetVelocityRadPerSec = velocityRadPerSec;
            state = FlywheelState.RAMP_UP;
        }), Set.of(this));
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
