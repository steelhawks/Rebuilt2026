package org.steelhawks.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class DoublePressTrigger extends Trigger {

    private final EventLoop m_loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    private final double debounce = 0.5; // in seconds
    private int counter;
    private final BooleanSupplier m_condition;

    public DoublePressTrigger(BooleanSupplier condition) {
        super(condition);
        this.m_condition = condition;
    }

    /**
     * Starts the given command whenever the condition changes from `false` to `true` twice within 100 ms.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onDoubleTap(Command command) {
        requireNonNullParam(command, "command", "onDoubleTap");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();
                private double m_pressedLastTime = Timer.getFPGATimestamp();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();
                    double currentTime = Timer.getFPGATimestamp();

                    if (currentTime - m_pressedLastTime > debounce) {
                        counter = 0;
                    }
                    if (!m_pressedLast && pressed) {
                        m_pressedLastTime = Timer.getFPGATimestamp();
                        counter++;
                    }
                    if (counter == 2) {
                        counter = 0;
                        CommandScheduler.getInstance().schedule(command);
                    }

                    m_pressedLast = pressed;
                }
            });
        return this;
    }
}
