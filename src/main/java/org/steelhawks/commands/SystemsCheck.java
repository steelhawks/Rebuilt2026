package org.steelhawks.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.RobotContainer;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.subsystems.indexer.Indexer;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.oldintake.OldIntake;
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

	public static Command swerve() {
		return Commands.runEnd(
			() -> DriveCommands.runVelocity(new Translation2d(2.0, 0.0), 2.0),
			s_Swerve::stop,
			s_Swerve);
		// zxeroes wheels, moves to 45, spins angular, spins linear
	}

	public static Command indexer() {
		return s_Indexer.feed();
	}

	public static Command intake() {
		return s_Intake.agitate()
			.alongWith(s_Intake.runIntake());
	}

	public static Command flywheel() {
		return s_Flywheel.setTargetVelocityForcedCmd(50.0);
	}

	public static Command hood() {
		return Commands.sequence(
			s_Hood.setDesiredPositionCommand(SubsystemConstants.OmegaBot.HOOD.minAngle()),
			Commands.waitUntil(s_Hood::atGoal),
			s_Hood.setDesiredPositionCommand(Hood.HOME_POSITION));
	}


	public static Command turret() {
		return Commands.sequence(
			s_Turret.setDesiredRotation(Rotation2d.fromRadians(-Math.PI / 2)),
			Commands.waitUntil(s_Turret::atGoal),
			s_Turret.setDesiredRotation(Rotation2d.fromRadians(Math.PI / 2)));
	}

}
