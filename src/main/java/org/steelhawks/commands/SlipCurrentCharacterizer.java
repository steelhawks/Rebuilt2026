

package org.steelhawks.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;

/**
 * A characterizer for the current for the wheels to start slipping when against a wall on carpet
 *
 * @author farhanjamil
 */
public class SlipCurrentCharacterizer extends Command {

    private final LinearFilter filter = LinearFilter.movingAverage(10);
    private final Timer timer = new Timer();
    private final double rampRate = 0.25; // m/s²
    private double commandedSpeed = 0.0;
    private double fracRamp = 0.0;
    private double currentStatorCurrent = 0.0;

    private final SwerveConsumer consumer;
    private final DoubleSupplier current;
    private final DoubleSupplier velocity;

    public SlipCurrentCharacterizer(
            SwerveConsumer consumer, DoubleSupplier statorCurrent, DoubleSupplier chassisVelocity) {
        this.consumer = consumer;
        this.current = statorCurrent;
        this.velocity = chassisVelocity;
    }

    @FunctionalInterface
    public interface SwerveConsumer {
        void accept(ChassisSpeeds chassisSpeeds);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        commandedSpeed = 0.0;
        currentStatorCurrent = 0.0;
    }

    @Override
    public void execute() {
        double dt = timer.get();
        timer.reset();
        timer.start();

        fracRamp = Math.min(fracRamp + rampRate * dt, 1.0);
        commandedSpeed = fracRamp * RobotContainer.s_Swerve.getMaxLinearSpeedMetersPerSec();
        currentStatorCurrent = current.getAsDouble();

        consumer.accept(new ChassisSpeeds(commandedSpeed, 0, 0));
        Logger.recordOutput("SlipCurrentCharacterizer/CommandedSpeed", commandedSpeed);
        Logger.recordOutput("SlipCurrentCharacterizer/ChassisVelocity", velocity.getAsDouble());
        Logger.recordOutput("SlipCurrentCharacterizer/ChassisStatorCurrent", currentStatorCurrent);
    }

    @Override
    public void end(boolean interrupted) {
        NumberFormat formatter = new DecimalFormat("#0.000");
        System.out.println("********** Slip Stator Current Characterization Results **********");
        System.out.println("\tStator Current: " + formatter.format(currentStatorCurrent) + " amps");
    }

    @Override
    public boolean isFinished() {
        return filter.calculate(velocity.getAsDouble()) > 0;
    }
}
