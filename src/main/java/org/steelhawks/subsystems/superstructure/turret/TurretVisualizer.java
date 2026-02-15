package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import java.util.function.Supplier;

public class TurretVisualizer {

	private final LoggedMechanism2d mTurret;
	private final LoggedMechanismLigament2d shooter;
	private final Supplier<Double> mTurretPosition;

	public TurretVisualizer(Supplier<Double> turretPosition) {
		this.mTurretPosition = turretPosition;
		mTurret = new LoggedMechanism2d(3, 3);
		var root = mTurret.getRoot("turretCenter", 1.5, 1.5);
		shooter = new LoggedMechanismLigament2d(
			"turret", 1, Math.toDegrees(turretPosition.get()) + 90, 5, new Color8Bit(Color.kWhite));
		root.append(shooter);
	}

	public void update() {
		shooter.setAngle(new Rotation2d(mTurretPosition.get() + Math.PI / 2));
		Logger.recordOutput("Turret/Mechanism2d", mTurret);

	}

}
