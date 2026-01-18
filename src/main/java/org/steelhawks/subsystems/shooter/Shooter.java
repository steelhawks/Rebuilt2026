package org.steelhawks.subsystems.shooter;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final ShooterIO shooterIO;
    private LoggedTunableNumber tuningVoltage;

    enum State {
        SPIN_UP,
        SAMPLING,
        HOLDING
    }

    private double[] samples = new double[ShooterConstants.NUMBER_OF_SAMPLES];
    private double sampleAverage = 0.0;
    private State currentPhase = State.SPIN_UP;
    private int currentSampleNum = 0;

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    @Override
    public void periodic() {
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (Toggles.tuningMode.get()) {
            if (Toggles.Shooter.shooterOpenLoopOverride.get()) {
                if (tuningVoltage == null) {
                    tuningVoltage = new LoggedTunableNumber("Shooter/TuningVolts", 0.0);
                }

                shooterIO.runOpenLoop(tuningVoltage.get());
            }

            LoggedTunableNumber.ifChanged(this.hashCode(),
                    () -> shooterIO.setPID(
                            ShooterConstants.kP.getAsDouble(),
                            ShooterConstants.kI.getAsDouble(),
                            ShooterConstants.kD.get()
                    ), ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD
            );
        }

        final boolean shouldRun =
                DriverStation.isEnabled()
                && Toggles.Shooter.shooterOpenLoopOverride.get();

        if (shouldRun) {
            switch (currentPhase) {
                case SPIN_UP -> {
                    shooterIO.runVelocity(inputs.velocityGoal, ShooterConstants.kS.get() + (ShooterConstants.kV.get() * inputs.velocityGoal));
                    if (isSpunUp()) {
                        // spun up, transition to sampling
                        currentPhase = State.SAMPLING;
                        currentSampleNum = 0;
                    }
                }
                case SAMPLING -> {
                    shooterIO.runVelocity(inputs.velocityGoal, ShooterConstants.kS.get() + (ShooterConstants.kV.get() * inputs.velocityGoal));
                    if (currentSampleNum < ShooterConstants.NUMBER_OF_SAMPLES && isSpunUp()) {
                        // add sample. only add the sample if the flywheel velocity is within range
                        samples[currentSampleNum] = inputs.motor1AppliedVolts;
                        currentSampleNum++;
                    }
                    if (currentSampleNum >= ShooterConstants.NUMBER_OF_SAMPLES) {
                        // samples collected, calculate average and start holding
                        sampleAverage = calculateAverageSampledVoltage();
                        currentPhase = State.HOLDING;
                    }
                }
                case HOLDING -> {
                    shooterIO.runOpenLoop(sampleAverage);
                }
            }
        } else {
            shooterIO.stop();
            currentPhase = State.HOLDING;
            sampleAverage = 0;
        }
    }

    private void stop() {
        shooterIO.stop();
    }

    public double getCurrentVelocity() {
        return inputs.motor1VelocityRadPerSec;
    }

    public boolean isSpunUp () {
        return Math.abs(inputs.velocityGoal - getCurrentVelocity()) < ShooterConstants.TOLERANCE;
    }

    private double calculateAverageSampledVoltage() {
        double sum = 0;
        for (double d : samples) {
            sum += d;
        }
        return sum / ShooterConstants.NUMBER_OF_SAMPLES;
    }

    public Command setVelocityRotPerSec(double velocityRotPerSec) {
        return Commands.runOnce(
                () -> {
                    if (!isSpunUp()) {
                        currentPhase = State.SPIN_UP;
                    }
                    inputs.velocityGoal = velocityRotPerSec;
                }
        );
    }
}

