package org.steelhawks.util;

import com.google.gson.InstanceCreator;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverWarnings extends VirtualSubsystem {
    private static DriverWarnings instance;

    public static class MomentaryAlert {
        private final Alert alert;
        private boolean active;
        private double lastTriggeredAt;
        private double autoDisableTime;

        public MomentaryAlert(String name, Alert.AlertType alertType) {
            alert = new Alert(name, alertType);
            active = false;
            lastTriggeredAt = -1;
            autoDisableTime = -1;
        }

        /**
         * Set the state of the alert
         * @param active Enable or disable the alert
         */
        public void set(boolean active) {
            if (active != this.active) {
                this.active = active;
                alert.set(active);
            }
        }

        /**
         * Enables this alert for a set amount of time, and then disables it automatically once this time has passes.
         *
         * You must run tick() periodically for this to work
         * @param autoDisableTime How long the alert should be active for, in seconds
         */
        public void triggerLapsing(double autoDisableTime) {
            lastTriggeredAt = Timer.getFPGATimestamp();
            this.autoDisableTime = autoDisableTime;
            if (!active) {
                set(true);
            }
        }

        /**
         * Run this method periodically to use the auto disable timer
         */
        public void tick() {
            if (autoDisableTime != -1 && lastTriggeredAt != -1 && active) {
                if (Timer.getFPGATimestamp() - lastTriggeredAt > autoDisableTime) {
                    set(false);
                    autoDisableTime = -1;
                    lastTriggeredAt = -1;
                }
            }
        }

        public boolean isActive() {
            return active;
        }

        public Trigger asTrigger() {
            return new Trigger(() -> this.isActive());
        }
    }

    public final MomentaryAlert tooFarAlert = new MomentaryAlert("Too far from Hub!", Alert.AlertType.kWarning);
    public final MomentaryAlert tooCloseAlert = new MomentaryAlert("Too close to Hub!", Alert.AlertType.kWarning);
    public final MomentaryAlert noSolutionAlert = new MomentaryAlert("Can't shoot! No solution", Alert.AlertType.kWarning);

    @Override
    public void periodic() {
        tooCloseAlert.tick();
        tooFarAlert.tick();
        LoopTimeUtil.record("DriverWarningsTick");
    }

    private DriverWarnings() {}

    public static DriverWarnings getInstance() {
        if (instance == null) {
            instance = new DriverWarnings();
        }
        return instance;
    }
}
