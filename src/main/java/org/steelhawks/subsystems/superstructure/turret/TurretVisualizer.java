package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.steelhawks.Constants;

import java.util.function.Supplier;

public class TurretVisualizer {

	private final LoggedMechanism2d turret;
	private final LoggedMechanismLigament2d shooter;
	private final Supplier<Double> turretPositionSupplier;
	private final Supplier<Pose2d> robotPositionSupplier;
	private Pose2d turretPose;

    private static final Translation2d TURRET_OFFSET_2D = new Translation2d(
        Constants.RobotConstants.ROBOT_TO_TURRET.getX(),
        Constants.RobotConstants.ROBOT_TO_TURRET.getY()
    );

	public TurretVisualizer(Supplier<Double> turretPosition, Supplier<Pose2d> robotPose) {
		this.turretPositionSupplier = turretPosition;
		this.robotPositionSupplier = robotPose;
		turret = new LoggedMechanism2d(3, 3);
		var root = turret.getRoot("TurretCenter", 1.5, 1.5);
		shooter = new LoggedMechanismLigament2d(
			"Turret", 1, Math.toDegrees(turretPosition.get()) + 90, 5, new Color8Bit(Color.kWhite));
		root.append(shooter);
		turretPose = new Pose2d(
			robotPose.get().getTranslation().plus(TURRET_OFFSET_2D),
			new Rotation2d(turretPositionSupplier.get() + Math.PI / 2));
	}

    public void update() {
        shooter.setAngle(new Rotation2d(turretPositionSupplier.get() + Math.PI / 2));
        Pose2d robot = robotPositionSupplier.get();
        Translation2d turretWorld = robot.getTranslation()
            .plus(TURRET_OFFSET_2D.rotateBy(robot.getRotation()));
        turretPose = new Pose2d(
            turretWorld,
            robot.getRotation().plus(new Rotation2d(turretPositionSupplier.get())));
        Logger.recordOutput("Turret/Mechanism2d", turret);
        Logger.recordOutput("Turret/TurretPose", turretPose);
        Logger.recordOutput("Turret/ComponentPoses", new Pose3d[] {
            new Pose3d(
                Constants.RobotConstants.ROBOT_TO_TURRET.getTranslation(),
                new Rotation3d(0.0, 0.0, turretPositionSupplier.get() + Math.PI)
            )
        });
    }
}
