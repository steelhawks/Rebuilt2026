package org.steelhawks.commands.rumble;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class RumbleAPI {

    private static CommandGenericHID[] controllers = new CommandGenericHID[0];

    public static void register(CommandGenericHID... controllers) {
        RumbleAPI.controllers = controllers;
    }

    /** Single steady rumble at full intensity for 1 second. */
    public static Command steady() {
        return new VibrateController(controllers);
    }

    /** Single steady rumble at the given intensity for 1 second. */
    public static Command steady(double intensity) {
        return new VibrateController(intensity, controllers);
    }

    /** Single steady rumble at the given intensity for the given duration. */
    public static Command steady(double intensity, double seconds) {
        return new VibrateController(intensity, seconds, controllers);
    }

    /** Staccato (pulsing) rumble with default settings. */
    public static Command staccato() {
        return new StacattoVibrationController(controllers);
    }

    /** Staccato rumble with custom parameters. */
    public static Command staccato(double intensity, double pulseTime, double pauseTime, double duration) {
        return new StacattoVibrationController(intensity, pulseTime, pauseTime, duration, controllers);
    }
}
