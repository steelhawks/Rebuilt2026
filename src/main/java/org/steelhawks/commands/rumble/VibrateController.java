package org.steelhawks.commands.rumble;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public class VibrateController extends Command {

    private static final double DEFAULT_VIBRATE_TIME = 1.0;

    private final CommandGenericHID[] controllers;
    private final double intensity, seconds;
    private double timer = 0.0;

    public static Command runOneShot(VibrateController controller) {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(controller));
    }

    public VibrateController(double intensity, double seconds, CommandGenericHID... controllers) {
        if (controllers.length == 0) {
            throw new IllegalArgumentException("At least one controller is required");
        }
        this.controllers = controllers;
        this.intensity = intensity;
        this.seconds = seconds;
    }

    public VibrateController(double intensity, CommandGenericHID... controllers) {
        this(intensity, DEFAULT_VIBRATE_TIME, controllers);
    }

    public VibrateController(CommandGenericHID... controllers) {
        this(1.0, DEFAULT_VIBRATE_TIME, controllers);
    }

    @Override
    public void initialize() {
        timer = 0.0;
        for (CommandGenericHID controller : controllers) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        }
    }

    @Override
    public void execute() {
        timer += 0.02;
    }

    @Override
    public boolean isFinished() {
        return timer >= seconds;
    }

    @Override
    public void end(boolean interrupted) {
        for (CommandGenericHID controller : controllers) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}