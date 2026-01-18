package org.steelhawks.subsystems.shooter;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {

	private final ShooterIO io;
	private final ShooterIOInputsAutoLogged inputs;
	private final TrapezoidProfile profile;
	private LoggedTunableNumber tuningVolts;

	private TrapezoidProfile.State currentState = new TrapezoidProfile.State();
	private TrapezoidProfile.State goal = new TrapezoidProfile.State();

	private boolean atGoal = false;

	public Shooter(ShooterIO io) {
		this.io = io;
		this.inputs = new ShooterIOInputsAutoLogged();
		profile =
			new TrapezoidProfile(
				new TrapezoidProfile.Constraints(
					ShooterConstants.MAX_VELOCITY_ROT_PER_SEC,
					ShooterConstants.MAX_ACCELERATION_ROT_PER_SEC_2
				)
			);
	}

	public double getVelocity() {
		return inputs.velocityRadPerSec;
	}

	public static double calculateDesiredVelocity(double distance) {
		return 0; // calculate using physics
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);

		if (DriverStation.isDisabled() && Robot.isFirstRun()) {
			io.setBrakeMode(false);
		}
		if (DriverStation.isEnabled()) {
			io.setBrakeMode(true);
		}

		if (Toggles.tuningMode.get()) {
			if (Toggles.Shooter.toggleVoltageOverride.getAsBoolean()) {
				if (tuningVolts == null) {
					tuningVolts = new LoggedTunableNumber("Shooter/TuningVolts", 0.0);
				}
				io.runOpenLoop(tuningVolts.get());
			}
			LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
				io.setPID(
					ShooterConstants.KP.get(),
					ShooterConstants.KI,
					ShooterConstants.KD);
			}, ShooterConstants.KP);
		}

		boolean shouldRun =
			DriverStation.isEnabled()
				&& !Toggles.Shooter.toggleVoltageOverride.getAsBoolean();

		Logger.recordOutput("Shooter/Running", shouldRun);
		inputs.shouldRunProfile = shouldRun;

		if (shouldRun) {
			inputs.goal =
				MathUtil.clamp(calculateDesiredVelocity(7), // find distance with vision, 7 is a placeholder
					0.0,
					ShooterConstants.MAX_VELOCITY_METERS_PER_SEC);
				;
			goal = new TrapezoidProfile.State(0.0, inputs.goal);
			currentState =
				profile
					.calculate(Constants.UPDATE_LOOP_DT, currentState, goal);
			atGoal = Math.abs(getVelocity() * ShooterConstants.FLYWHEEL_RADIUS - goal.velocity) <= ShooterConstants.TOLERANCE;
			if (atGoal) {
				io.stop();
			} else {
				io.runOpenLoop(inputs.goal * ShooterConstants.kV + ShooterConstants.kS);
			}

			Logger.recordOutput("Shooter/StatePosition", currentState.position);
			Logger.recordOutput("Shooter/StateVelocity", currentState.velocity);
			Logger.recordOutput("Shooter/GoalPosition", goal.position);
			Logger.recordOutput("Shooter/GoalVelocity", goal.velocity);
		}

		Logger.recordOutput("Shooter/AtGoal", atGoal);
	}
}

