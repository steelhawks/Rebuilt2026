package org.steelhawks.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.Robot;
import org.steelhawks.RobotContainer;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

	private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private TrapezoidProfile profile;
	private final IntakeIO io;

	private LoggedTunableNumber tuningVolts;
	private LoggedTunableNumber tuningAmps;

	private IntakeConstants.State desiredGoal = IntakeConstants.State.HOME;
	private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
	private TrapezoidProfile.State goal = new TrapezoidProfile.State();
	private boolean brakeModeEnabled = false;
	private boolean atGoal = false;

	public Intake(IntakeIO io) {
		this.io = io;
		profile =
			new TrapezoidProfile(
				new TrapezoidProfile.Constraints(
					IntakeConstants.MAX_VELOCITY_RAD_PER_SEC.get(),
					IntakeConstants.MAX_ACCEL_RAD_PER_SEC_SQ.get()));
	}

    public boolean atGoal() {
        return atGoal;
    }

	public Rotation2d getPosition() {
        return inputs.encAbsPositionRad;
    }

	private void setBrakeMode(boolean enabled) {
		if (brakeModeEnabled == enabled) return;
		brakeModeEnabled = enabled;
		io.setBrakeMode(brakeModeEnabled);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		final boolean shouldRun =
			DriverStation.isEnabled()
			&& (inputs.leftConnected && inputs.rightConnected && inputs.encConnected)
			&& Toggles.Intake.isEnabled.get()
			&& !Toggles.Intake.toggleCurrentOverride.get()
			&& !Toggles.Intake.toggleVoltageOverride.get()
			&& (getPosition().getRadians() >= IntakeConstants.MIN_ROTATION.getRadians()
				&& getPosition().getRadians() <= IntakeConstants.MAX_ROTATION.getRadians());

		if (DriverStation.isDisabled()) {
			setpoint = new TrapezoidProfile.State(getPosition().getRadians(), 0.0);
		}
		if (DriverStation.isDisabled() && Robot.isFirstRun()) {
			setBrakeMode(false);
		}
		if (DriverStation.isEnabled()) {
			setBrakeMode(true);
		}
		if (Toggles.tuningMode.get()) {
			if (Toggles.Intake.toggleVoltageOverride.get()) {
				if (tuningVolts == null) {
					tuningVolts = new LoggedTunableNumber("Intake/TuningVolts", 0.0);
				}
				io.runPivotOpenLoop(tuningVolts.get(), false);
			}
			if (Toggles.Intake.toggleCurrentOverride.get()) {
				if (tuningAmps == null) {
					tuningAmps = new LoggedTunableNumber("Intake/TuningAmps", 0.0);
				}
				io.runPivotOpenLoop(tuningAmps.get(), true);
			}
			LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
				io.setPivotPID(
				IntakeConstants.kP.get(),
				IntakeConstants.kI.get(),
				IntakeConstants.kD.get());
			}, IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
			if (IntakeConstants.MAX_VELOCITY_RAD_PER_SEC.hasChanged(hashCode())
			    || IntakeConstants.MAX_ACCEL_RAD_PER_SEC_SQ.hasChanged(hashCode())
            ) {
				profile =
					new TrapezoidProfile(
						new TrapezoidProfile.Constraints(
							IntakeConstants.MAX_VELOCITY_RAD_PER_SEC.get(),
							IntakeConstants.MAX_ACCEL_RAD_PER_SEC_SQ.get()));
			}
		}
		if (shouldRun) {
			if (RobotContainer.s_Swerve.collisionDetected()) {
				setDesiredState(IntakeConstants.State.RETRACTED);
			}
			double previousVelocity = setpoint.velocity;
			setpoint =
				profile.calculate(Constants.UPDATE_LOOP_DT, setpoint, goal);
			if (setpoint.position < IntakeConstants.MIN_ROTATION.getRadians()
			    || setpoint.position > IntakeConstants.MAX_ROTATION.getRadians()
            ) {
				setpoint =
					new TrapezoidProfile.State(
						MathUtil.clamp(
							setpoint.position,
							IntakeConstants.MIN_ROTATION.getRadians(),
							IntakeConstants.MAX_ROTATION.getRadians()),
						0.0);
			}
			atGoal = Math.abs(getPosition().getRadians() - goal.position) <= IntakeConstants.TOLERANCE;
			if (atGoal) {
				io.stopPivot();
			} else {
				double acceleration = (setpoint.velocity - previousVelocity) / Constants.UPDATE_LOOP_DT;
                double rawAccelY = RobotContainer.s_Swerve.getRobotRelativeYAccelGs();
                double drivetrainAccelG = rawAccelY - Math.sin(RobotContainer.s_Swerve.getPitch().getRadians());
                double drivetrainAccel = drivetrainAccelG * 9.81;
                double kT = DCMotor.getKrakenX44Foc(2).KtNMPerAmp * IntakeConstants.REDUCTION;
				io.runPivotPosition(
					setpoint.position,
				(IntakeConstants.kG.get() * Math.cos(getPosition().getRadians())) +
                    -(IntakeConstants.kMassRadius.getAsDouble() * drivetrainAccel * Math.sin(getPosition().getRadians())) / kT +
					IntakeConstants.kS.get() * Math.signum(setpoint.velocity) +
					IntakeConstants.kA.get() * acceleration);
			}
			Logger.recordOutput("Intake/SetpointPosition", setpoint.position);
			Logger.recordOutput("Intake/SetpointVelocity", setpoint.velocity);
			Logger.recordOutput("Intake/GoalPosition", goal.position);
			Logger.recordOutput("Intake/GoalVelocity", goal.velocity);
		} else {
			setpoint = new TrapezoidProfile.State(getPosition().getRadians(), 0.0);
			Logger.recordOutput("Intake/SetpointPosition", 0.0);
			Logger.recordOutput("Intake/SetpointVelocity", 0.0);
			Logger.recordOutput("Intake/GoalPosition", 0.0);
			Logger.recordOutput("Intake/GoalVelocity", 0.0);
		}
	}

	public void setDesiredState(IntakeConstants.State state) {
        inputs.goal = MathUtil.clamp(
            state.getPosition().getRadians(),
            IntakeConstants.MIN_ROTATION.getRadians(),
            IntakeConstants.MAX_ROTATION.getRadians());
        goal = new TrapezoidProfile.State(inputs.goal, 0.0);
        desiredGoal = state;
	}

    public Command setDesiredStateCommand(IntakeConstants.State state) {
        return Commands.runOnce(() -> setDesiredState(state), this);
    }

	public Command runIntake() {
		return Commands.run(
			() -> {
				io.runIntake(1.0);
			}, this).finallyDo(io::stopIntake);
	}
}

