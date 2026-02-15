package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import java.util.function.Supplier;

public class TurretVisualizer {

	private final LoggedMechanism2d turret;
	private final LoggedMechanismLigament2d shooter;
	private final Supplier<Double> turretPositionSupplier;
	private final Supplier<Pose2d> robotPositionSupplier;
	private Pose2d turretPose;
	private final Translation2d ROBOT_TO_TURRET = new Translation2d(Units.inchesToMeters(-6.7), 0); // hahaha

	public TurretVisualizer(Supplier<Double> turretPosition, Supplier<Pose2d> robotPose) {
		this.turretPositionSupplier = turretPosition;
		this.robotPositionSupplier = robotPose;
		turret = new LoggedMechanism2d(3, 3);
		var root = turret.getRoot("turretCenter", 1.5, 1.5);
		shooter = new LoggedMechanismLigament2d(
			"turret", 1, Math.toDegrees(turretPosition.get()) + 90, 5, new Color8Bit(Color.kWhite));
		root.append(shooter);

		turretPose = new Pose2d(
			robotPose.get().getTranslation().plus(ROBOT_TO_TURRET),
			new Rotation2d(turretPositionSupplier.get() + Math.PI / 2));
	}

	public void update() {
		shooter.setAngle(new Rotation2d(turretPositionSupplier.get() + Math.PI / 2));
		turretPose = new Pose2d(
			robotPositionSupplier.get().getTranslation().plus(ROBOT_TO_TURRET),
			new Rotation2d(turretPositionSupplier.get()).plus(robotPositionSupplier.get().getRotation()));
		Logger.recordOutput("Turret/Mechanism2d", turret);
		Logger.recordOutput("Turret/TurretPose", turretPose);
		// aidan ortiz was here
	}
}
