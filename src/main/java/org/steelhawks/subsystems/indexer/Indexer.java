package org.steelhawks.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.beam.BeamIO;
import org.steelhawks.subsystems.beam.BeamIOInputsAutoLogged;
import org.steelhawks.util.BatteryUtil;
import org.steelhawks.util.LoggedTunableNumber;

public class Indexer extends SubsystemBase {

    private static LoggedTunableNumber SPINDEXER_JAM_CURRENT;
    private static LoggedTunableNumber FEEDER_JAM_CURRENT;
    private static LoggedTunableNumber BEAM_DEBOUNCE_TIME;
    private static LoggedTunableNumber FEEDER_STALL_DEBOUNCE;
    private static LoggedTunableNumber SPINDEXER_STALL_DEBOUNCE;

    private LoggedTunableNumber tuningSpindexerVolts;
    private LoggedTunableNumber tuningFeederVolts;

    private Debouncer beamDebouncer;
    private Debouncer feederStallDebouncer;
    private Debouncer spindexerStallDebouncer;

    private boolean feederStalledCache = false;
    private boolean spindexerStalledCache = false;

    public enum IndexerState {
        RUNNING(1.0, 1.0),
        OUTTAKING(-0.6, -1.0);

        final double spindexerOutput;
        final double feederOutput;

        IndexerState(double spindexerOutput, double feederOutput) {
            this.spindexerOutput = spindexerOutput;
            this.feederOutput = feederOutput;
        }
    }

    private final SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();
    private final FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
    private final BeamIOInputsAutoLogged beamInputs = new BeamIOInputsAutoLogged();
    private final IndexerIO io;
    private final BeamIO beamIO;

    public Indexer(IndexerIO io, BeamIO beamIO, SubsystemConstants.IndexerConstants constants) {
        SPINDEXER_JAM_CURRENT =
            new LoggedTunableNumber("Indexer/Spindexer/JamCurrent", 55.0);
        FEEDER_JAM_CURRENT =
            new LoggedTunableNumber("Indexer/Feeder/JamCurrent", 60.0);
        BEAM_DEBOUNCE_TIME =
            new LoggedTunableNumber("Indexer/Beam/DebounceTime", 0.3);
        FEEDER_STALL_DEBOUNCE =
            new LoggedTunableNumber("Indexer/Feeder/StallDebounceTime", 0.1);
        SPINDEXER_STALL_DEBOUNCE =
            new LoggedTunableNumber("Indexer/Spindexer/StallDebounceTime", 0.15);

        this.io = io;
        this.beamIO = beamIO;

        beamDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kFalling);
        feederStallDebouncer = new Debouncer(FEEDER_STALL_DEBOUNCE.get(), Debouncer.DebounceType.kRising);
        spindexerStallDebouncer = new Debouncer(SPINDEXER_STALL_DEBOUNCE.get(), Debouncer.DebounceType.kRising);
    }

    @Override
    public void periodic() {
        io.updateInputs(spindexerInputs, feederInputs);
        beamIO.updateInputs(beamInputs);
        Logger.processInputs("Indexer/Spindexer/Inputs", spindexerInputs);
        Logger.processInputs("Indexer/Feeder/Inputs", feederInputs);
        Logger.processInputs("Indexer/Beam/Inputs", beamInputs);
        BatteryUtil.recordCurrentUsage("Feeder", feederInputs.currentAmps);
        BatteryUtil.recordCurrentUsage(
            "Spindexer",
            spindexerInputs.motor1CurrentAmps + spindexerInputs.motor2CurrentAmps);
        feederStalledCache = feederStallDebouncer.calculate(
            feederInputs.torqueCurrentAmps >= FEEDER_JAM_CURRENT.get());
        spindexerStalledCache = spindexerStallDebouncer.calculate(
            spindexerInputs.motor1TorqueCurrentAmps >= SPINDEXER_JAM_CURRENT.get());

        if (feederStalledCache) {
            io.stopSpindexer();
        }

        if (Toggles.tuningMode.get()) {
            if (Toggles.Indexer.toggleSpindexerVoltageOverride.get()) {
                if (tuningSpindexerVolts == null) {
                    tuningSpindexerVolts = new LoggedTunableNumber("Indexer/Spindexer/TuningVolts", 0.0);
                }
                io.runSpindexer(tuningSpindexerVolts.get());
            }
            if (Toggles.Indexer.toggleFeederVoltageOverride.get()) {
                if (tuningFeederVolts == null) {
                    tuningFeederVolts = new LoggedTunableNumber("Indexer/Feeder/TuningAmps", 0.0);
                }
                io.runFeeder(tuningFeederVolts.get());
            }
        }
    }

    @AutoLogOutput(key = "Indexer/ShouldRun")
    private boolean shouldRun() {
        return Toggles.Indexer.isFeederEnabled.get()
            && Toggles.Indexer.isSpindexerEnabled.get()
            && !Toggles.tuningMode.get();
    }

    @AutoLogOutput(key = "Indexer/FeederStalled")
    public boolean isFeederStalled() {
        return feederStalledCache;
    }

    @AutoLogOutput(key = "Indexer/SpindexerStalled")
    public boolean isSpindexerStalled() {
        return spindexerStalledCache;
    }

    @AutoLogOutput(key = "Indexer/Jammed")
    public boolean isJammed() {
        return feederStalledCache || spindexerStalledCache;
    }

    private boolean beamHasEverDetected = false;

    @AutoLogOutput(key = "Indexer/HopperEmpty")
    public boolean emptyFuel() {
        boolean detected = beamInputs.detected;
        if (detected) beamHasEverDetected = true;
        if (!beamHasEverDetected) return false;
        return !beamDebouncer.calculate(detected);
    }

    public void resetBeamState() {
        beamDebouncer = new Debouncer(BEAM_DEBOUNCE_TIME.get(), Debouncer.DebounceType.kFalling);
        beamHasEverDetected = false;
    }

    public Command runSpindexer() {
        return Commands.runEnd(
            () -> io.runSpindexer(IndexerState.RUNNING.spindexerOutput),
            io::stopSpindexer,
            this)
        .onlyIf(this::shouldRun);
    }

    public Command runFeeder() {
        return Commands.runEnd(
            () -> io.runFeeder(IndexerState.RUNNING.feederOutput),
            io::stopFeeder,
            this)
        .onlyIf(this::shouldRun);
    }

    public Command feed() {
        return Commands.runEnd(
            () -> {
                io.runFeeder(IndexerState.RUNNING.feederOutput);
                if (!feederStalledCache) {
                    io.runSpindexer(IndexerState.RUNNING.spindexerOutput);
                } else {
                    io.stopSpindexer();
                }
            },
            () -> {
                io.stopSpindexer();
                io.stopFeeder();
            },
            this)
        .onlyIf(this::shouldRun);
    }

    public Command outtake() {
        return Commands.runEnd(
            () -> {
                io.runSpindexer(IndexerState.OUTTAKING.spindexerOutput);
                io.runFeeder(IndexerState.OUTTAKING.feederOutput);
            },
            () -> {
                io.stopSpindexer();
                io.stopFeeder();
            },
            this)
        .onlyIf(this::shouldRun);
    }

    public Command feedWithJamRecovery() {
        return Commands.repeatingSequence(
            feed().until(this::isJammed),
            Commands.either(
                Commands.sequence(
                    outtake().withTimeout(0.3),
                    Commands.waitSeconds(0.05),
                    feed().withTimeout(0.2)
                ),
                Commands.none(),
                this::isJammed
            )
        ).onlyIf(this::shouldRun);
    }

    public Command agitateSpindexer() {
        return Commands.repeatingSequence(
            Commands.runEnd(
                () -> io.runSpindexer(IndexerState.OUTTAKING.spindexerOutput),
                io::stopSpindexer,
                this).withTimeout(0.1),
            Commands.runEnd(
                () -> io.runSpindexer(IndexerState.RUNNING.spindexerOutput),
                io::stopSpindexer,
                this).withTimeout(0.1)
        ).onlyIf(this::shouldRun);
    }
}
