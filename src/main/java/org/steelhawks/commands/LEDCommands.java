package org.steelhawks.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.*;
import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;

import static org.steelhawks.Robot.RobotState.AUTON;
import static org.steelhawks.Robot.RobotState.TELEOP;

public class LEDCommands {

    private static final LEDMatrix s_Matrix = RobotContainer.s_Matrix;
    private final LEDTriggers triggers;

    public LEDCommands(Trigger toggleMatchData) {
        s_Matrix.clearCommand();
        triggers = new LEDTriggers(toggleMatchData);
    }

    public static Command requestMatchDataScreen() {
        return Commands.runOnce(() ->
            s_Matrix.playAnimation(new LEDMatrix.ScrollingText(
                "",
                Color.WHITE,
                1)
                ), s_Matrix
            ).andThen(
            Commands.run(() -> {
                String current = RobotState.getInstance().isOurHubActive() ? "Hub Active" : "Hub Inactive";
                s_Matrix.updateText(current);
            })
            ).ignoringDisable(true);
    }

    public static Command runTechnicianScreen() {
        return Commands.runOnce(() ->
            s_Matrix.playAnimation(new LEDMatrix.ScrollingText(
                "No Auton Selected",
                Color.WHITE,
                1)
            ), s_Matrix
        ).andThen(
            Commands.run(() -> {
                String current = Autos.getAuto() == null
                    ? "No Auton Selected"
                    : "Auton Selected: " + Autos.getAuto().getName();
                s_Matrix.updateText(current);
            })
        ).ignoringDisable(true);
    }

    public static Command displayTimeLeftInShift() {
        return Commands.runOnce(() ->
            s_Matrix.playAnimation(new LEDMatrix.StaticText(
            "",
            Color.WHITE)), s_Matrix)
        .andThen(
            Commands.run(() -> {
                int currentOnesNum = (int) RobotState.getInstance().timeLeftInShift();
                String currentOnes = String.valueOf(currentOnesNum);
                int currentDecimalsNum = (int) ((RobotState.getInstance().timeLeftInShift() - (int) RobotState.getInstance().timeLeftInShift()) * 100);
                String currentDecimals = String.valueOf(currentDecimalsNum);
                if (currentDecimalsNum < 10) {
                    currentDecimals = "0" + currentDecimalsNum;
                }
                String current = String.format(currentOnes + ":" + currentDecimals);
                s_Matrix.updateText(current);
            }))
        .ignoringDisable(true);
    }

    public static Command runSteelHawks() {
        return Commands.sequence(
            Commands.runOnce(() -> s_Matrix.playAnimation(new LEDMatrix.StaticText(
                "", Color.WHITE)), s_Matrix),
            Commands.run(() -> s_Matrix.updateText("STEEL")).withTimeout(2.0),
            s_Matrix.clearCommand(),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> s_Matrix.playAnimation(new LEDMatrix.StaticText(
                "", Color.WHITE)), s_Matrix),
            Commands.run(() -> s_Matrix.updateText("HAWKS")).withTimeout(2.0),
            Commands.waitSeconds(1.0))
        .repeatedly()
        .ignoringDisable(true);
    }

    private static class LEDTriggers {

        private final Trigger runTechnicianScreen;
        private final Trigger runRainbowLEDs;
        private final Trigger warn10Seconds;

        private final Trigger isAuton;
        private final Trigger isTeleop;

        private LEDTriggers(Trigger toggleMatchData) {

            runTechnicianScreen = new Trigger(() -> Robot.isFirstRun() && DriverStation.isDisabled()).debounce(10)
                .whileTrue(runTechnicianScreen())
                .onFalse(s_Matrix.clearCommand());

            runRainbowLEDs = new Trigger(() -> !Robot.isFirstRun() && DriverStation.isDisabled())
                .whileTrue(s_Matrix.rainbowWaveCommand(10))
                .onFalse(s_Matrix.clearCommand());

            warn10Seconds = new Trigger(() -> RobotState.getInstance().isShift() && RobotState.getInstance().timeLeftInShift() < 10.0)
                .whileTrue(displayTimeLeftInShift())
                .onFalse(s_Matrix.clearCommand());

            toggleMatchData
                .whileTrue(requestMatchDataScreen())
                .onFalse(s_Matrix.clearCommand());

            isAuton = new Trigger(() -> Robot.getState().equals(AUTON))
                .whileTrue(
                    runSteelHawks());

            isTeleop = new Trigger(() -> Robot.getState().equals(TELEOP))
                    .and(runTechnicianScreen.negate())
                    .and(runRainbowLEDs.negate())
                    .and(warn10Seconds.negate())
                    .and(toggleMatchData.negate())
                .whileTrue(s_Matrix.fireCommand(5, 2));

        }
    }
}

