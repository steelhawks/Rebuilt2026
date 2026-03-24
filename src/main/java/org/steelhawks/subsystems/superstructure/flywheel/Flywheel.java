package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;

    private FlywheelState flywheelState = FlywheelState.IDLE;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    enum FlywheelState {
        RAMP_UP,
        RUNNING,
        IDLE,
    }

    // -----------------------------------------------------------------------
    // Flywheel state
    // -----------------------------------------------------------------------
    private double targetFlywheelVelocity = 0.0;
    private final double flywheelVelocityTolerance = 5.0; // rad/s

    /** Computed average of velocity samples collected during the RAMP_UP window. */
    private double sampledVelocity = 0.0;

    // Ramp-up averaging
    private final Timer rampTimer = new Timer();
    private double velocitySampleSum = 0.0;
    private int velocitySampleCount = 0;
    private static final double RAMP_UP_DURATION_SECONDS = 2.0;

    private BuilderConstants.FlywheelConstants constants;

    // PID / FF tunables
    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kV;

    /**
     * Simple voltage applied in the IDLE state so the flywheel keeps a slow
     * spin (reduces spin-up time on demand) without using a sampled velocity.
     */
    private static LoggedTunableNumber idleVolts;

    public Flywheel(FlywheelIO io, BuilderConstants.FlywheelConstants constants) {
        this.io = io;
        this.constants = constants;

        kP = new LoggedTunableNumber("Flywheel/kP", constants.kP());
        kI = new LoggedTunableNumber("Flywheel/kI", constants.kI());
        kD = new LoggedTunableNumber("Flywheel/kD", constants.kD());
        kS = new LoggedTunableNumber("Flywheel/kS", constants.kS());
        kV = new LoggedTunableNumber("Flywheel/kV", constants.kV());
        idleVolts = new LoggedTunableNumber("Flywheel/IdleVolts", 0.0);

        inputs = new FlywheelIOInputsAutoLogged();
    }

    // -----------------------------------------------------------------------
    // periodic
    // -----------------------------------------------------------------------

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        Logger.recordOutput("Flywheel/State", flywheelState.toString());

        final boolean shouldRun = Toggles.Flywheel.isEnabled.get();

        // ---- Tuning overrides (only active in tuning mode) -----------------
        if (Toggles.tuningMode.getAsBoolean()) {

            LoggedTunableNumber.ifChanged(
                    hashCode(), () -> io.setFlywheelPID(kP.get(), kI.get(), kD.get()), kP, kI, kD
            );

            if (Toggles.Flywheel.toggleVoltageOverride.getAsBoolean()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Flywheel/TuningVolts", 0.0);
                }
                io.runOpenLoop(tuningVolts.getAsDouble(), false);
            }

            if (Toggles.Flywheel.toggleCurrentOverride.getAsBoolean()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Flywheel/TuningAmps", 0.0);
                }
                io.runOpenLoop(tuningAmps.getAsDouble(), true);
            }
        }

        // Feedforward based on the *current* target (used by RAMP_UP)
        double feedforward = kS.get() + kV.get() * targetFlywheelVelocity;
        Logger.recordOutput("Flywheel/FeedForward", feedforward);

        // ---- State machine --------------------------------------------------
        if (shouldRun) {
            switch (flywheelState) {

                // ---- IDLE --------------------------------------------------
                // Run at a simple, constant idle voltage — not sample-based.
                case IDLE -> {
                    io.runOpenLoop(idleVolts.get(), false);
                }

                // ---- RAMP_UP -----------------------------------------------
                // Run closed-loop toward targetFlywheelVelocity for exactly
                // RAMP_UP_DURATION_SECONDS, accumulating a velocity average
                // every loop tick. After the window closes the average is
                // stored as sampledVelocity and we transition to RUNNING.
                case RAMP_UP -> {
                    io.runFlywheel(targetFlywheelVelocity, feedforward, false);

                    // Accumulate samples
                    velocitySampleSum += inputs.velocityRadPerSec;
                    velocitySampleCount++;

                    if (rampTimer.hasElapsed(RAMP_UP_DURATION_SECONDS)) {
                        // Compute and store the rolling average
                        sampledVelocity = (velocitySampleCount > 0)
                                ? velocitySampleSum / velocitySampleCount
                                : targetFlywheelVelocity;

                        Logger.recordOutput("Flywheel/SampledVelocity", sampledVelocity);
                        flywheelState = FlywheelState.RUNNING;
                    }
                }

                // ---- RUNNING -----------------------------------------------
                // If a valid sample was collected, run closed-loop at that
                // averaged velocity. If sampledVelocity is still zero (e.g.
                // the flywheel never spun during ramp), fall back to open-loop
                // feedforward so the motor at least produces some output.
                case RUNNING -> {
                    if (sampledVelocity != 0.0) {
                        double sampledFF = kS.get() + kV.get() * sampledVelocity;
                        io.runFlywheel(sampledVelocity, sampledFF, false);
                    } else {
                        io.runOpenLoop(feedforward, false);
                    }
                }
            }
        }

        Logger.recordOutput("Flywheel/State", flywheelState.toString());
        Logger.recordOutput("Flywheel/TargetVelocity", targetFlywheelVelocity);
        Logger.recordOutput("Flywheel/SampledVelocity", sampledVelocity);
        Logger.recordOutput("Flywheel/RampElapsedSec", rampTimer.get());
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /**
     * Snapshot the current velocity, set it as the target, and reset all
     * ramp-up accumulators + timer.  Call this before entering RAMP_UP.
     */
    private void beginRampUp(double targetVelocityRadPerSec) {
        targetFlywheelVelocity = targetVelocityRadPerSec;
        velocitySampleSum  = 0.0;
        velocitySampleCount = 0;
        rampTimer.reset();
        rampTimer.start();
    }

    // -----------------------------------------------------------------------
    // Public accessors
    // -----------------------------------------------------------------------

    public double getSampledVelocity() {
        return sampledVelocity;
    }

    public double getVelocity() {
        return inputs.velocityRadPerSec;
    }

    /**
     * Returns true once the ramp-up window has elapsed and we are in RUNNING.
     * Useful as a WaitUntil condition for command sequences.
     */
    public boolean isRampComplete() {
        return flywheelState == FlywheelState.RUNNING;
    }

    /** Legacy at-target check retained for callers that still need it. */
    public boolean isAtTarget() {
        return (inputs.velocityRadPerSec >= targetFlywheelVelocity - flywheelVelocityTolerance);
    }

    // -----------------------------------------------------------------------
    // Command Factories
    // -----------------------------------------------------------------------

    /**
     * Ramp up from the flywheel's current live velocity for 2 seconds,
     * average the samples, then hold at the resulting sampled velocity.
     */
    public Command runAtSampleVelocity() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    beginRampUp(inputs.velocityRadPerSec); // target = current live velocity
                    flywheelState = FlywheelState.RAMP_UP;
                }, this),
                Commands.waitUntil(this::isRampComplete)
        ).withName("Flywheel.runAtSampledVelocity");
    }


    /**
     * Ramp up to the requested velocity for 2 seconds, average actual
     * velocity samples, then run at the sampled average.
     */
    public void runAtVelocity(double velocityRadPerSec) {
        System.out.println("Reached the RunAtVelocity Function");
        beginRampUp(velocityRadPerSec);
        flywheelState = FlywheelState.RAMP_UP;
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            flywheelState = FlywheelState.IDLE;
            sampledVelocity = 0.0;
            io.stopFlywheel();
        }, this).withName("Flywheel.stop");
    }
}