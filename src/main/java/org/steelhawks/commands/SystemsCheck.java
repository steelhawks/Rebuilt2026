package org.steelhawks.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.superstructure.flywheel.Flywheel;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.subsystems.superstructure.turret.Turret;
import org.steelhawks.subsystems.swerve.Swerve;

public final class SystemsCheck {

	private static final Swerve s_Swerve = RobotContainer.s_Swerve;
	private static final Indexer s_Indexer = RobotContainer.s_Indexer;
	private static final Intake s_Intake = RobotContainer.s_Intake;
	private static final Flywheel s_Flywheel = RobotContainer.s_Flywheel;
	private static final Hood s_Hood = RobotContainer.s_Hood;
	private static final Turret s_Turret = RobotContainer.s_Turret;

	private SystemsCheck() {
		throw new RuntimeException("This is a util class.");
	}

	public static Command zeroSwerve() {
		return Commands.runOnce(() -> s_Swerve.runAngle(Rotation2d.kZero), s_Swerve);
	}

	public static Command testSwerveAngle() {
		return Commands.runOnce(() -> s_Swerve.runAngle(Rotation2d.fromDegrees(45.0)), s_Swerve);
	}

	public static Command testSwerveDriving() {
		return Commands.run(() -> s_Swerve.runDriveCharacterization(3.0), s_Swerve)
			.finallyDo(s_Swerve::stop);
	}

	public static Command testSwerveTurning() {
		return Commands.run(() -> s_Swerve.runTurnCharacterization(3.0), s_Swerve)
			.finallyDo(s_Swerve::stop);
	}

	public static Command spindexer() {
		return s_Indexer.runSpindexer();
	}

	public static Command feeder() {
		return s_Indexer.runFeeder();
	}

	public static Command intake() {
		return s_Intake.agitate()
			.alongWith(s_Intake.runIntake());
	}

	public static Command flywheel() {
		return s_Flywheel.setTargetVelocityForcedCmd(50.0)
			.finallyDo(() -> s_Flywheel.setTargetVelocity(0.0));
	}

	public static Command hood() {
		return Commands.sequence(
			s_Hood.setDesiredPositionCommand(SubsystemConstants.OmegaBot.HOOD.minAngle()),
			Commands.waitUntil(s_Hood::atGoal),
			s_Hood.setDesiredPositionCommand(Hood.HOME_POSITION)); // needs testing
	}

	public static Command turret() {
		return Commands.sequence(
			s_Turret.setDesiredRotation(Rotation2d.fromRadians(-Math.PI / 2)),
			Commands.waitUntil(s_Turret::atGoal),
			s_Turret.setDesiredRotation(Rotation2d.fromRadians(Math.PI / 2)));
	}
}
