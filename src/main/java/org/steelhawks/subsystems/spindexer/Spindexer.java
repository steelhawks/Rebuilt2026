package org.steelhawks.subsystems.spindexer;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Robot;
import org.steelhawks.Toggles;
import org.steelhawks.util.AlertUtil;
import org.steelhawks.util.LoggedTunableNumber;

public class Spindexer extends SubsystemBase {

    private static final LoggedTunableNumber SPINDEXER_JAM_CURRENT =
        new LoggedTunableNumber("Spindexer/JamCurrent", 40.0); // TODO tune
    private static final double WHEEL_RADIUS = Units.inchesToMeters(4.0);

	public enum SpindexerState {
		RUNNING(0.8),
		STOPPED(0.0),
		OUTTAKING(-0.8);

		final double output;

		SpindexerState(double output) {
			this.output = output;
		}
	}

	private SpindexerState desiredState = SpindexerState.STOPPED;
	private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
	private final SpindexerIO io;

	public Spindexer(SpindexerIO io) {
		this.io = io;
        new AlertUtil("[Spindexer]: Motor disconnected.", AlertType.kError)
            .withCondition(() -> !inputs.connected);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Spindexer/Inputs", inputs);
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
			io.stop();
		}
		Logger.recordOutput("Spindexer/DesiredState", desiredState.toString());
	}

	private void setDesiredState(SpindexerState state) {
		desiredState = state;
	}

	public Command setDesiredStateCmd(SpindexerState state) {
		return Commands.runOnce(() -> setDesiredState(state), this);
	}
}
