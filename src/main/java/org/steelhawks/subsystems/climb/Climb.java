package org.steelhawks.subsystems.climb;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import kotlin.DeepRecursiveFunction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

import java.util.ConcurrentModificationException;

public class Climb extends SubsystemBase {
    private final Debouncer hominhDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final ClimbIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;
    private Rotation2d desiredRotation = new Rotation2d();
    private boolean brakeModeEnabled = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;

    public Climb(ClimbIO io) {
        this.io = io;
    }

    private double getPosition() {
        return inputs.positionRad;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    @AutoLogOutput(key = "Climb/AtGoal")
    public boolean atGoal() {
        return atGoal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
        if (!isHomed && Toggles.Climb.isEnabled.get()) {
            io.runPercentOut(ClimbConstants.HOMING_VOLTS);
            isHomed = hominhDebouncer.calculate(inputs.supplyCurrentAmps > 40);
            Logger.recordOutput("Climb/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(0);
                io.stop();
                isZeroed = true;
                desiredRotation = new Rotation2d();
                Logger.recordOutput("Climb/Zeroed", true);
            }
        }

    }
}

