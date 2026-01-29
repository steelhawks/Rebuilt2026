package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.Set;

import static edu.wpi.first.units.Units.Volts;

public class Flywheel extends SubsystemBase {

    public static final int motorId = 0;
    public static final LoggedTunableNumber kP =
        new LoggedTunableNumber("Flywheel/kP", 0.0);
    public static final LoggedTunableNumber kI =
        new LoggedTunableNumber("Flywheel/kI", 0.0);
    public static final LoggedTunableNumber kD =
        new LoggedTunableNumber("Flywheel/kD", 0.0);
    public static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Flywheel/kS", 0.0);
    public static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Flywheel/kV", 0.0);
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
        SAMPLE_UNTIL_READY,
        RAMP_UP,
        HOLD
    }

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private FlywheelState state = FlywheelState.HOLD;
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

        nearTargetVelocity = Math.abs(inputs.velocityRadPerSec - targetVelocityRadPerSec) <= velocityTolerance.get();
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
            if (Toggles.Flywheel.toggleAdaptiveFeedforward.get()) {
                switch (state) {
                    case RAMP_UP -> {
                        io.runFlywheel(targetVelocityRadPerSec, kS.get() + kV.get() * targetVelocityRadPerSec, false);
                        if (nearTargetVelocity) {
                            state = FlywheelState.SAMPLE_UNTIL_READY;
                            currentSampleIndex = 0;
                        }
                    }
                    case SAMPLE_UNTIL_READY -> {
                        io.runFlywheel(targetVelocityRadPerSec, kS.get() + kV.get() * targetVelocityRadPerSec, false);
                        if (nearTargetVelocity && currentSampleIndex < sampleCounts) {
                            voltageSamples[currentSampleIndex] = inputs.appliedVolts;
                            currentSampleIndex++;
                        }
                        if (currentSampleIndex >= sampleCounts) {
                            sampledVoltage = calculateAverageSample();
                            state = FlywheelState.HOLD;
                            Logger.recordOutput("Flywheel/SampledVoltage", sampledVoltage);
                        }
                    }
                    case HOLD -> {
                        io.runFlywheelOpenLoop(sampledVoltage, false);
                    }
                }
            } else {
                io.runFlywheel(targetVelocityRadPerSec, kS.get() + kV.get() * targetVelocityRadPerSec, false);
            }
        } else {
            state = FlywheelState.HOLD;
            sampledVoltage = 0.0;
        }
        Logger.recordOutput("Flywheel/State", state.toString());
        Logger.recordOutput("Flywheel/TargetVelocity", targetVelocityRadPerSec);
    }

    private double calculateAverageSample() {
        double sum = 0.0;
        for (double sample : voltageSamples) {
            sum += sample;
        }
        return sum / sampleCounts;
    }

    public boolean isReadyToShoot() {
        return state == FlywheelState.HOLD && nearTargetVelocity;
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command setTargetVelocity(double velocityRadPerSec) {
        return Commands.defer(() -> Commands.runOnce(() -> {
            if (Math.abs(targetVelocityRadPerSec - velocityRadPerSec) > 1.0) {
                targetVelocityRadPerSec = velocityRadPerSec;
                state = FlywheelState.RAMP_UP;
            }
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
