package org.steelhawks.util;

import com.google.gson.InstanceCreator;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;

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

        public void set(boolean active) {
            if (active != this.active) {
                this.active = active;
                alert.set(active);
            }
        }

        public void triggerLapsing(double autoDisableTime) {
            lastTriggeredAt = Timer.getFPGATimestamp();
            this.autoDisableTime = autoDisableTime;
            if (!active) {
                set(true);
            }
        }

        public void tick() {
            if (autoDisableTime != -1 && lastTriggeredAt != -1 && active) {
                if (Timer.getFPGATimestamp() - lastTriggeredAt > autoDisableTime) {
                    set(false);
                    autoDisableTime = -1;
                    lastTriggeredAt = -1;
                }
            }
        }
    }

    public static final MomentaryAlert tooFarAlert = new MomentaryAlert("Too far from Hub!", Alert.AlertType.kWarning);
    public static final MomentaryAlert tooCloseAlert = new MomentaryAlert("Too close to Hub!", Alert.AlertType.kWarning);

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
