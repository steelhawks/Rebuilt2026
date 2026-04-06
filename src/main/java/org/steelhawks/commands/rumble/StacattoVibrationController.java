package org.steelhawks.commands.rumble;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class StacattoVibrationController extends Command {
    public static final double DEFAULT_DURATION = 2;
    public static final double DEFAULT_PAUSE_TIME = 0.5;
    public static final double DEFAULT_PULSE_TIME = 0.5;

    private CommandGenericHID[] controllers;
    private final double intensity;
    private final double pulseTime;
    private final double pauseTime;
    private final double duration;

    private double timer = 0;
    private boolean rumbling = true;
    private double phaseTimer = 0;

    /**
     * Vibrates in short pulses separated by an interval
     *
     * @param intensity The intensity to vibrate at
     * @param pulseTime The "on" time of each pulse
     * @param pauseTime The "off" time in between pulses
     * @param duration The vibration will end after this duration regardless of the phase it is in
     * @param controllers The controllers to vibrate. You need at least one.
     */
    public StacattoVibrationController(double intensity, double pulseTime, double pauseTime, double duration, CommandGenericHID... controllers) {
        if (controllers.length == 0) {
            throw new IllegalArgumentException("At least one controller is required");
        }
        this.controllers = controllers;
        this.intensity = intensity;
        this.pulseTime = pulseTime;
        this.pauseTime = pauseTime;
        this.duration = duration;
        setName("StacattoVibrationController");
    }

    public StacattoVibrationController(CommandGenericHID... controllers) {
        this(1, DEFAULT_PULSE_TIME, DEFAULT_PAUSE_TIME, DEFAULT_DURATION, controllers);
    }

    @Override
    public void initialize() {
        for (CommandGenericHID controller : controllers) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        }
    }

    @Override
    public void execute() {
        timer += 0.02;
        phaseTimer += 0.02;

        if (rumbling && phaseTimer >= pulseTime) {
            for (CommandGenericHID controller : controllers) {
                controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            }
            rumbling = false;
            phaseTimer = 0.0;
        } else if (!rumbling && phaseTimer >= pauseTime) {
            for (CommandGenericHID controller : controllers) {
                controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
            }

            rumbling = true;
            phaseTimer = 0.0;
        }
    }

    @Override
    public boolean isFinished() {
        return timer >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        for (CommandGenericHID controller : controllers) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}
