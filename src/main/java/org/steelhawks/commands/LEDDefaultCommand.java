package org.steelhawks.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.Robot;
import org.steelhawks.subsystems.led.LEDMatrix;

public class LEDDefaultCommand extends Command {

    private final static int rainbowSpeed = 5;

    private final LEDMatrix s_LED;

    private final Trigger runTechnicianScreen;
    private final Trigger runRainbowLEDs;

    public Command requestMatchDataScreen() {
        return Commands.run(() -> {
            System.out.println("RUNNING MATCH LED SCREEN");
        }, s_LED).finallyDo(s_LED::clear);
    }

    public LEDDefaultCommand(LEDMatrix s_LED) {
        this.s_LED = s_LED;

        runTechnicianScreen =
            new Trigger(() -> Robot.isFirstRun() && DriverStation.isDisabled())
                .whileTrue(Commands.runOnce(() -> System.out.println("TESTING"), s_LED));

        runRainbowLEDs = new Trigger(() -> !Robot.isFirstRun() && DriverStation.isDisabled())
            .whileTrue(s_LED.rainbowWaveCommand(rainbowSpeed));
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        super.execute();
    }
}
