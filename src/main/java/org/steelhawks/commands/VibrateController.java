package org.steelhawks.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * Class to simplify controlling the vibration of controllers
 *
 * @author farhanj2
 */
public class VibrateController extends InstantCommand {

    private static final double DEFAULT_VIBRATE_TIME = 1;
    private final Timer timer = new Timer();

    private final CommandGenericHID[] controllers;
    private final double intensity, seconds;

    /**
     * Constructs a command that rumbles the controller.
     *
     * @param controllers the controllers to vibrate
     * @param intensity  the intensity to rumble the controller at between 0.0 and 1.0
     * @param seconds    the amount of time to vibrate the controller for
     */
    public VibrateController(double intensity, double seconds, CommandGenericHID... controllers) {

        this.controllers = controllers;
        this.intensity = intensity;
        this.seconds = seconds;
    }

    /**
     * Constructs a command that rumbles the controller at a set intensity for 1 second.
     *
     * @param controllers the controllers to vibrate
     * @param intensity  the intensity to rumble the controller at between 0.0 and 1.0
     */
    public VibrateController(double intensity, CommandGenericHID... controllers) {
        this(intensity, DEFAULT_VIBRATE_TIME, controllers);
    }

    /**
     * Constructs a command that rumbles the controller at full intensity for 1 second.
     *
     * @param controllers the controllers to vibrate
     */
    public VibrateController(CommandGenericHID... controllers) {
        this(1.0, DEFAULT_VIBRATE_TIME, controllers);
    }

    @Override
    public void initialize() {
        for (CommandGenericHID controller : controllers) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        }
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();

        for (CommandGenericHID controller : controllers) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}
