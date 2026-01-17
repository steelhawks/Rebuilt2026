package org.steelhawks.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * This class is used to characterize a mechanism by running it at different
 * voltages and recording the resulting velocities.
 * <p>
 * The kV value is calculated using the formula:
 * kV = (v2 - v1) / (u2 - u1)
 * where v is the velocity and u is the voltage.
 *
 * @author Farhan Jamil
 */
public class FeedforwardCharacterize {

    // everything is UNTESTED
    public static Command runkV(
        DoubleConsumer voltageSetter, DoubleSupplier encoderPosition, DoubleSupplier encoderVelocity, SubsystemBase subsystem, double minPosition, double maxPosition) {
        List<Double> voltages = List.of(0.5, 1.0, 1.5, 2.0, 2.5);
        List<Double> velocities = new ArrayList<>();
        List<Double> recordedVoltages = new ArrayList<>();

        return Commands.sequence(
            Commands.runOnce(() -> voltageSetter.accept(0), subsystem),
            Commands.sequence(
                voltages.stream().map(voltage ->
                    Commands.sequence(
                        Commands.runOnce(() -> {
                            double position = encoderPosition.getAsDouble();
                            if (position >= maxPosition) return;
                            voltageSetter.accept(voltage);
                            recordedVoltages.add(voltage);
                        }, subsystem),
                        Commands.waitSeconds(1.0), // time to let mechanism stabilize itsself
                        Commands.runOnce(() -> velocities.add(encoderVelocity.getAsDouble()), subsystem)
                    )
                ).toArray(Command[]::new)
            ),
            // reverse movement
            Commands.sequence(
                voltages.stream().sorted((a, b) -> Double.compare(b, a)).map(voltage ->
                    Commands.sequence(
                        Commands.runOnce(() -> {
                            double position = encoderPosition.getAsDouble();
                            if (position <= minPosition) return;
                            voltageSetter.accept(-voltage);
                        }, subsystem),
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> velocities.add(-encoderVelocity.getAsDouble()), subsystem)
                    )
                ).toArray(Command[]::new)
            ),
            Commands.runOnce(() -> voltageSetter.accept(0), subsystem),
            Commands.runOnce(() -> {
                double kV = calculateKV(recordedVoltages, velocities);
                System.out.println("Calculated kV: " + kV);
            })
        );
    }

    private static double calculateKV(List<Double> voltages, List<Double> velocities) {
        double sumSlope = 0;
        int count = voltages.size() - 1;
        for (int i = 0; i < count; i++) {
            double slope = (velocities.get(i + 1) - velocities.get(i)) / (voltages.get(i + 1) - voltages.get(i));
            sumSlope += slope;
        }
        return sumSlope / count;
    }
}
