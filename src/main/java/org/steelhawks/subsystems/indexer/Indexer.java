package org.steelhawks.subsystems.indexer;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

public class Indexer extends SubsystemBase {

    private static final LoggedTunableNumber INDEXER_JAM_CURRENT =
        new LoggedTunableNumber("Indexer/JamCurrent", 40.0); // TODO tune
    private static final double WHEEL_RADIUS = Units.inchesToMeters(4.0);

	public enum IndexerState {
		RUNNING(0.6, -1.0),
		STOPPED(0.0, 0.0),
		OUTTAKING(-0.6, 1.0);

		final double spindexerOutput;
        final double feederOutput;

		IndexerState(double spindexerOutput, double feederOutput) {
			this.spindexerOutput = spindexerOutput;
            this.feederOutput = feederOutput;
		}
	}

	private IndexerState desiredState = IndexerState.STOPPED;
	private final SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

	private final IndexerIO io;

	public Indexer(IndexerIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(spindexerInputs, feederInputs);
		Logger.processInputs("Indexer/Spindexer/Inputs", spindexerInputs);
        Logger.processInputs("Indexer/Feeder/Inputs", feederInputs);
		final boolean shouldRun =
			spindexerInputs.connected &&
            feederInputs.connected &&
			DriverStation.isEnabled() &&
			Toggles.Indexer.isSpindexerEnabled.getAsBoolean() &&
            Toggles.Indexer.isFeederEnabled.getAsBoolean();
		if (shouldRun) {
			if (spindexerInputs.torqueCurrentAmps >= INDEXER_JAM_CURRENT.get()
                || feederInputs.torqueCurrentAmps >= INDEXER_JAM_CURRENT.get()) {
				setDesiredState(IndexerState.OUTTAKING);
			} else {
				setDesiredState(IndexerState.RUNNING);
			}
			io.runSpindexer(desiredState.spindexerOutput);
            io.runFeeder(desiredState.feederOutput);
		} else {
			setDesiredState(IndexerState.STOPPED);
			io.stopSpindexer();
            io.stopFeeder();
		}
		Logger.recordOutput("Indexer/DesiredState", desiredState.toString());
	}

	private void setDesiredState(IndexerState state) {
		desiredState = state;
	}

	public Command setDesiredStateCmd(IndexerState state) {
		return Commands.runOnce(() -> setDesiredState(state), this);
	}
}
