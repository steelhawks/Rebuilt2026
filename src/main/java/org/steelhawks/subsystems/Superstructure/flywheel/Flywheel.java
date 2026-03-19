package org.steelhawks.subsystems.Superstructure.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;

    private FlywheelState flywheelState = FlywheelState.IDLE;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    enum FlywheelState {
        RAMP_UP,
        RUNNING,
        IDLE,
    }

    private FlywheelConstants constants;

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kV;

    public Flywheel(FlywheelIO io, FlywheelConstants constants) {
        this.io = io;
        this.constants = constants;


        kP = new LoggedTunableNumber("Flywheel/kP", constants.kP);
        kI = new LoggedTunableNumber("Flywheel/kI", constants.kI);
        kD = new LoggedTunableNumber("Flywheel/kD", constants.kD);
        kS = new LoggedTunableNumber("Flywheel/kS", constants.kS);
        kV = new LoggedTunableNumber("Flywheel/kV", constants.kV);

        inputs = new FlywheelIOInputsAutoLogged();

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        Logger.recordOutput("Flywheel/State", flywheelState.toString());


        if (Toggles.tuningMode.getAsBoolean()) {

            LoggedTunableNumber.ifChanged(
                    hashCode(), () -> io.setFlywheelPID(kP.get(), kI.get(), kD.get()), kP, kI, kD
            );

            if (Toggles.FLywheel.toggleVoltageOverride.getAsBoolean()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Flywheel/TuningVolts", 0.0);
                }
                io.runOpenLoop(tuningVolts.getAsDouble(), false);
            }

            if (Toggles.FLywheel.toggleCurrentOverride.getAsBoolean()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Flywheel/TuningAmps", 0.0);
                }
                io.runOpenLoop(tuningAmps.getAsDouble(), true);
            }

        }
    }

}
