

package org.steelhawks.util;

import edu.wpi.first.wpilibj.Alert;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public final class AlertUtil {

    private BooleanSupplier condition;
    private final Alert alert;

    private static final ArrayList<AlertUtil> alerts = new ArrayList<>();
    private static final VirtualSubsystem updater =
            new VirtualSubsystem("AlertUtil") {
                @Override
                public void periodic() {
                    for (AlertUtil alertUtil : alerts) {
                        alertUtil.update();
                    }
                }
            };

    public AlertUtil(String message, Alert.AlertType alertType) {
        alert = new Alert(message, alertType);
        alerts.add(this);
    }

    public Alert withCondition(BooleanSupplier condition) {
        this.condition = condition;
        return alert;
    }

    public boolean isActive() {
        return alert.get();
    }

    public void updateCondition(BooleanSupplier condition) {
        this.condition = condition;
    }

    private void update() {
        alert.set(condition != null && condition.getAsBoolean());
    }
}
