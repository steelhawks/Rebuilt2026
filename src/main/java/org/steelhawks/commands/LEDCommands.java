package org.steelhawks.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.*;
import org.steelhawks.subsystems.led.Color;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.led.anim.AnimationLibrary;

import static org.steelhawks.Robot.RobotState.AUTON;
import static org.steelhawks.Robot.RobotState.TELEOP;

public class LEDCommands {

    private static final LEDMatrix s_Matrix = RobotContainer.s_Matrix;
    private LEDCommands() {}

    public static void configureTriggers(Trigger toggleMatchData) {
        new LEDTriggers(toggleMatchData);
    }

    public static Command requestMatchDataScreen() {
        return Commands.runOnce(() ->
            s_Matrix.playAnimation(new LEDMatrix.ScrollingText(
                "",
                Color.WHITE,
                1)
                ), s_Matrix)
            .andThen(
                Commands.run(() -> {
                    String current = RobotState.getInstance().isOurHubActive() ? "Hub Active" : "Hub Inactive";
                    s_Matrix.updateText(current);
                })
            ).ignoringDisable(true);
    }

    private static final int ARROW_START_X = 16;
    private static final int ARROW_REGION_W = 16;
    private static final double MIN_INTERVAL = 0.08;
    private static final double MAX_INTERVAL = 0.7;

    private static void updateArrowInterval(double interval) {
        if (s_Matrix.getOverlayAnimation() instanceof LEDMatrix.DirectionalArrow da) {
            da.setOscillateInterval(interval);
        }
    }

    private static double getOscillateInterval(double errorMagnitude, double maxError) {
        double clamped = Math.min(errorMagnitude, maxError);
        return MIN_INTERVAL + (clamped / maxError) * (MAX_INTERVAL - MIN_INTERVAL);
    }

    private static LEDMatrix.DirectionalArrow.Direction getArrowDirection(Autos.Misalignment m) {
        return switch (m) {
            case X_LEFT, ROTATION_CCW -> LEDMatrix.DirectionalArrow.Direction.LEFT;
            case X_RIGHT, ROTATION_CW -> LEDMatrix.DirectionalArrow.Direction.RIGHT;
            case Y_FORWARD -> LEDMatrix.DirectionalArrow.Direction.UP;
            case Y_BACKWARD -> LEDMatrix.DirectionalArrow.Direction.DOWN;
            case MULTIPLE, NONE -> LEDMatrix.DirectionalArrow.Direction.NONE;
        };
    }

    public static Command runTechnicianWizard() {
        return Commands.sequence(
        Commands.runOnce(() -> s_Matrix.setBrightness(0.25)),
            // No Auton Selected warning
                s_Matrix.clearCommand(),
                Commands.waitSeconds(0.3),
                Commands.runOnce(() -> s_Matrix.playAnimation(new LEDMatrix.StaticText("NO", Color.RED)), s_Matrix),
                Commands.waitSeconds(1.0),
                s_Matrix.flashCommand(Color.BLACK, 0.0, 0.5),
                Commands.runOnce(() -> s_Matrix.playAnimation(new LEDMatrix.StaticText("AUTON", Color.RED)), s_Matrix),
                Commands.waitSeconds(1.0),
                s_Matrix.flashCommand(Color.BLACK, 0.0, 0.5),
                Commands.runOnce(() -> s_Matrix.playAnimation(new LEDMatrix.StaticText("SELECT", Color.RED)), s_Matrix),
                Commands.waitSeconds(1.0),
                s_Matrix.flashCommand(Color.BLACK, 0.0, 0.5)
            ).repeatedly().until(() -> Autos.getAuto() != null),

            // Show name of seelcted auton once
            Commands.runOnce(() -> {
                s_Matrix.clearOverlay();
                s_Matrix.playAnimation(new LEDMatrix.ScrollingText("", Color.WHITE, 1));
            }, s_Matrix),
            Commands.run(() -> s_Matrix.updateText(Autos.getAuto().getName()))
                .withTimeout(4.0),

            s_Matrix.clearCommand(),
            Commands.waitSeconds(0.3),

            // alignment wizard
            Commands.runOnce(() -> {
                s_Matrix.playAnimation(new LEDMatrix.ScrollingText("", Color.WHITE, 2, 0, ARROW_START_X));
                s_Matrix.playOverlay(new LEDMatrix.DirectionalArrow(
                    LEDMatrix.DirectionalArrow.Direction.NONE, MAX_INTERVAL, Color.YELLOW,
                    ARROW_START_X, ARROW_REGION_W));
            }, s_Matrix),
            Commands.run(() -> {
                Autos.Misalignment misalignment = Autos.getMisalignment();
                String label = switch (misalignment) {
                    case NONE -> "ALIGNED";
                    case X_LEFT -> "GO LEFT";
                    case X_RIGHT -> "GO RIGHT";
                    case Y_FORWARD -> "GO FWD";
                    case Y_BACKWARD -> "GO BACK";
                    case ROTATION_CW -> "ROT CW";
                    case ROTATION_CCW -> "ROT CCW";
                    case MULTIPLE -> "ADJUST";
                };
                Color textColor = misalignment == Autos.Misalignment.NONE ? Color.GREEN : Color.RED;
                s_Matrix.updateText(label);

                LEDMatrix.DirectionalArrow.Direction dir = getArrowDirection(misalignment);
                if (!(s_Matrix.getOverlayAnimation() instanceof LEDMatrix.DirectionalArrow da)
                    || da.getDirection() != dir) {
                    s_Matrix.playOverlay(new LEDMatrix.DirectionalArrow(
                        dir, MAX_INTERVAL, textColor, ARROW_START_X, ARROW_REGION_W));
                } else {
                    double errorMag = switch (misalignment) {
                        case NONE -> 0.0;
                        case MULTIPLE -> 1.0;
                        default -> 0.5;
                    };
                    updateArrowInterval(getOscillateInterval(errorMag, 1.0));
                }
            }).until(() -> Autos.getAuto() == null),
            Commands.runOnce(s_Matrix::clearOverlay),
            s_Matrix.clearCommand())
        .repeatedly().ignoringDisable(true);
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

        private LEDTriggers(Trigger toggleMatchData) {
            Trigger runTechnicianScreen = new Trigger(() -> Robot.isFirstRun() && DriverStation.isDisabled()).debounce(1)
                .whileTrue(runTechnicianWizard())
                .onFalse(s_Matrix.clearCommand());

            Trigger runRainbowLEDs = new Trigger(() -> !Robot.isFirstRun() && DriverStation.isDisabled())
                .whileTrue(s_Matrix.rainbowWaveCommand(10))
                .onFalse(s_Matrix.clearCommand());

            Trigger warn10Seconds = new Trigger(() -> RobotState.getInstance().isShift() && RobotState.getInstance().timeLeftInShift() < 10.0)
                .whileTrue(displayTimeLeftInShift())
//                    .beforeStarting(() -> s_Matrix.setBrightness(1.0)))
                .onFalse(s_Matrix.clearCommand());

            toggleMatchData
                .whileTrue(requestMatchDataScreen())
                .onFalse(s_Matrix.clearCommand());

            Trigger isAuton = new Trigger(() -> Robot.getState().equals(AUTON))
                .whileTrue(s_Matrix.plasmaCommand(100));
//                    .beforeStarting(() -> s_Matrix.setBrightness(1.0)));

            Trigger isTeleop = new Trigger(() -> Robot.getState().equals(TELEOP))
                    .and(runTechnicianScreen.negate())
                    .and(runRainbowLEDs.negate())
                    .and(warn10Seconds.negate())
                    .and(toggleMatchData.negate())
                .whileTrue(s_Matrix.fireCommand(15, 20));
        }
    }
}

