package org.steelhawks.subsystems.superstructure.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    class TurretIOInputs {}

    default void updateInputs(TurretIOInputs inputs) {}
}
