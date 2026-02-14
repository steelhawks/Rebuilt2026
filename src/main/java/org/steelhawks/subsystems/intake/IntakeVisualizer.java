package org.steelhawks.subsystems.intake;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.function.Supplier;

public class IntakeVisualizer {

    private Supplier<Double> rackPositionSupplier;
    private LoggedMechanism2d mechanism;
    private LoggedMechanismLigament2d rack;
    private LoggedMechanismRoot2d root;

    public IntakeVisualizer(
        Supplier rackPositionSupplier, double rackLength, double rackAngle
    ) {
        this.rackPositionSupplier = rackPositionSupplier;
        mechanism = new LoggedMechanism2d(3, 3);
        root = mechanism.getRoot("pinion", 2, 2);

        rack = root.append(new LoggedMechanismLigament2d(
            "rack",
            rackLength,
            rackAngle,
            10,
            new Color8Bit(Color.kWhite)
        ));
    }

    public void update() {
        rack.setLength(rackPositionSupplier.get());
        Logger.recordOutput("Intake/Mechanism", mechanism);
    }
}
