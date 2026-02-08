package org.steelhawks.subsystems.spindexer;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

public class Spindexer extends SubsystemBase {

	public enum SpindexerState {
		RUNNING(0.8),
		STOPPED(0.0),
		OUTTAKING(-0.8);

		final double output;

		SpindexerState(double output) {
			this.output = output;
		}
	}

	private boolean brakeModeEnabled;

	private SpindexerState desiredState = SpindexerState.STOPPED;

	private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
	private final SpindexerIO io;

	private final LoggedTunableNumber SPINDEXER_JAM_CURRENT = new LoggedTunableNumber("Spindexer/JamCurrent", 0.0);

	public Spindexer(SpindexerIO io) {
		this.io = io;
	}

	public void setBrakeMode(boolean enabled) {
		if (brakeModeEnabled == enabled) return;
		io.setBrakeMode(enabled);
		brakeModeEnabled = enabled;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Spindexer/Inputs", inputs);

		if (DriverStation.isDisabled() && Robot.isFirstRun()) {
			setBrakeMode(false);
		}
		if (DriverStation.isEnabled()) {
			setBrakeMode(true);
		}
		final boolean shouldRun =
			inputs.connected &&
			DriverStation.isEnabled() &&
			Toggles.Spindexer.isEnabled.getAsBoolean();

		if (shouldRun) {
			if (inputs.torqueCurrentAmps >= SPINDEXER_JAM_CURRENT.get()) {
				setDesiredState(SpindexerState.OUTTAKING);
			} else {
				setDesiredState(SpindexerState.RUNNING);
			}
			io.runSpindexer(desiredState.output);
		} else {
			setDesiredState(SpindexerState.STOPPED);
			io.stopSpindexer();
		}

		Logger.recordOutput("Spindexer/DesiredState", desiredState.toString());
	}

	public void setDesiredState(SpindexerState state) {
		desiredState = state;
	}

	public Command setDesiredStateCommand(SpindexerState state) {
		return Commands.runOnce(() -> setDesiredState(state), this);
	}
}

