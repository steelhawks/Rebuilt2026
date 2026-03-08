package org.steelhawks.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Faults {

    public enum ClearBehavior {
        IMMEDIATE,
        LATCH
    }

    public static final class Fault {
        public final String id;
        public final String message;
        public final Alert.AlertType severity;
        public final ClearBehavior clearBehavior;
        public final BooleanSupplier supplier;

        boolean active;

        public Fault(String id, String message, Alert.AlertType severity, ClearBehavior clearBehavior, BooleanSupplier supplier) {
            this.id = id;
            this.message = message;
            this.severity = severity;
            this.clearBehavior = clearBehavior;
            this.supplier = supplier;
        }
    }

    private static Faults instance;
    private static final Map<String, Fault> registry = new LinkedHashMap<>();
    private static final Map<String, Alert> ntAlerts = new LinkedHashMap<>();
    public static final double NT_PUSH_FREQUENCY = 0.5;
    private double lastUpdate = Timer.getFPGATimestamp();

    private Faults() {}

    public static Faults getInstance() {
        if (instance == null) {
            instance = new Faults();
        }

        return instance;
    }

    public void registerFault(String id, String message, Alert.AlertType severity, ClearBehavior clearBehavior, BooleanSupplier supplier) {
        Fault newFault = new Fault(
            id,
            message,
            severity,
            clearBehavior,
            supplier
        );
        registry.put(id, newFault);
        ntAlerts.put(id, new Alert(message, severity));
    }

    public void update() {
        for (var entry : registry.values()) {
            if (entry.supplier != null) {
                if (entry.supplier.getAsBoolean()) {
                    entry.active = true;
                } else {
                    if (entry.clearBehavior == ClearBehavior.IMMEDIATE) {
                        entry.active = false;
                    }
                }
            }
        }

        if (Timer.getFPGATimestamp() - lastUpdate > NT_PUSH_FREQUENCY ) {
            for (var entry : registry.values()) {
                var alert = ntAlerts.get(entry.id);
                alert.set(entry.active);
            }
        }

        lastUpdate = Timer.getFPGATimestamp();
    }

    public List<Fault> getActive() {
        return registry.values().stream()
            .filter(e -> e.active)
            .toList();
    }

    public void set(String id, boolean active) {
        Fault fault = registry.get(id);
        if (active) {
            fault.active = true;
        } else {
            if (fault.clearBehavior == ClearBehavior.IMMEDIATE) {
                fault.active = false;
            }
        }
    }
}
