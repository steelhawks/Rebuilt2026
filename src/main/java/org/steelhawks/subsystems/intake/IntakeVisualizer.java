package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.function.Supplier;

public class IntakeVisualizer {
    private final Supplier<Double> rackPositionSupplier;
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d rack;

    public IntakeVisualizer(
            Supplier<Double> rackPositionSupplier, double rackLength, double rackAngle
    ) {
        this.rackPositionSupplier = rackPositionSupplier;
        mechanism = new LoggedMechanism2d(3, 3);
        LoggedMechanismRoot2d root = mechanism.getRoot("pinion", 2, 2);
        rack = root.append(new LoggedMechanismLigament2d(
                "rack", rackLength, rackAngle, 10, new Color8Bit(Color.kWhite)
        ));
    }

    public void update() {
        double positionMeters = rackPositionSupplier.get();
        rack.setLength(positionMeters);
        Logger.recordOutput("Intake/Mechanism", mechanism);
        Logger.recordOutput("Intake/ComponentPoses", new Pose3d[]{
                new Pose3d(
                        new Translation3d(
                                positionMeters * Math.cos(IntakeConstants.EXTENSION_ANGLE),
                                0.0,
                                positionMeters * Math.sin(IntakeConstants.EXTENSION_ANGLE)
                        ),
                        new Rotation3d()
                )
        });
    }
}
