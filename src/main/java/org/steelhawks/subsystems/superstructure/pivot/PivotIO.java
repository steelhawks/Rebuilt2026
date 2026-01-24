package org.steelhawks.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    class PivotIOInputs {}

    default void updateInputs(PivotIOInputs inputs) {}
}
