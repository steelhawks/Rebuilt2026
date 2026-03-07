package org.steelhawks.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;

public class LEDCommands {

    private final LEDMatrix s_Matrix;
    private final LEDTriggers triggers;

    public LEDCommands(LEDMatrix s_Matrix) {
        this.s_Matrix = s_Matrix;
        triggers = new LEDTriggers();
    }

    public Command requestMatchDataScreen() {
        return Commands.run(() -> {
            System.out.println("RUNNING MATCH LED SCREEN");
        }, s_Matrix).finallyDo(s_Matrix::clear);
    }

    public Command runTechnicianScreen() {
        return Commands.run(() ->
            s_Matrix.scrollingTextCommand(
                "Selected Auton: ",
                Color.WHITE,
                5
            ), s_Matrix).finallyDo(s_Matrix::clear);
    }

    public Command runRainbowLEDs() {
        return Commands.run(() ->
            s_Matrix.rainbowWaveCommand(5
            ), s_Matrix).finallyDo(s_Matrix::clear);
    }

    private static class LEDTriggers {

        private Trigger runTechnicianScreen;
        private Trigger runRainbowLEDs;

        public LEDTriggers() {

//            runTechnicianScreen = new Trigger(() -> Robot.isFirstRun() && DriverStation.isDisabled()).debounce(10)
//                .whileTrue(runTechnicianScreen());
//
//            runRainbowLEDs = new Trigger(() -> !Robot.isFirstRun() && DriverStation.isDisabled())
//                .whileTrue(runRainbowLEDs());
        }
    }
}

