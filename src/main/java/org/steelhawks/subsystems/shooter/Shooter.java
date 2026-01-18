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
            boolean atGoal = Math.abs(inputs.velocityGoal - getCurrentVelocity()) < ShooterConstants.TOLERANCE;
            if (atGoal) {
                shooterIO.runVelocity(inputs.velocityGoal, ShooterConstants.kS.get() + (ShooterConstants.kV.get() * inputs.velocityGoal));
            } else  {
                shooterIO.runOpenLoop(ShooterConstants.kS.getAsDouble() + (ShooterConstants.kV.getAsDouble() * inputs.velocityGoal));
            }
        } else {
            shooterIO.stop();
        }
    }

    private void stop() {
        shooterIO.stop();
    }

    public double getCurrentVelocity() {
        return inputs.velocityRadPerSec;
    }

    public boolean isSpunUp () {
        return Math.abs(inputs.velocityGoal - getCurrentVelocity()) < ShooterConstants.TOLERANCE;
    }

    public Command setVelocityRotPerSec(double velocityRotPerSec) {
        return Commands.runOnce(
                () -> shooterIO.runVelocity(velocityRotPerSec,
                        ShooterConstants.kS.getAsDouble() + (ShooterConstants.kV.getAsDouble() * inputs.velocityGoal)
                )
        );
    }
}

