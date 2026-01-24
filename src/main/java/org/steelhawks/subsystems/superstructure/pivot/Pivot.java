package org.steelhawks.subsystems.superstructure.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final PivotIO io;

    public Pivot(PivotIO io) {
        this.io = io;
    }
}
