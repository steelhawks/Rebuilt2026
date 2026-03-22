package org.steelhawks.subsystems.Superstructure.flywheel;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;
import org.steelhawks.BuilderConstants;
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

    // Flywheel state
    private double targetFlywheelVelocity = 0.0;
    private double flywheelVelocityTolerance = 5.0; // rad/s

    private static LoggedTunableNumber targetVelocityTunable;
    private double sampledVelocity;


    private BuilderConstants.FlywheelConstants constants;

    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber kV;

    public Flywheel(FlywheelIO io, BuilderConstants.FlywheelConstants constants) {
        this.io = io;
        this.constants = constants;


        kP = new LoggedTunableNumber("Flywheel/kP", constants.kP());
        kI = new LoggedTunableNumber("Flywheel/kI", constants.kI());
        kD = new LoggedTunableNumber("Flywheel/kD", constants.kD());
        kS = new LoggedTunableNumber("Flywheel/kS", constants.kS());
        kV = new LoggedTunableNumber("Flywheel/kV", constants.kV());

        targetVelocityTunable = new LoggedTunableNumber("Flywheel/Target Velocity", 0.0);

        inputs = new FlywheelIOInputsAutoLogged();

    }

    @Override
    public void periodic() {
        System.out.println("Reached Periodic");
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        Logger.recordOutput("Flywheel/State", flywheelState.toString());

        final boolean shouldRun = Toggles.FLywheel.isEnabled.get() == true;

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

        double feedforward = kS.get() * Math.signum(targetFlywheelVelocity) + kV.get() * targetFlywheelVelocity;
        Logger.recordOutput("Flywheel/FeedForward", feedforward);

        System.out.println("Should run State" + shouldRun);

        if (shouldRun) {
            switch (flywheelState) {
                case IDLE -> {
                    io.stopFlywheel();
                }

                case RAMP_UP -> {
                    System.out.println("RAMP_UP REACHED" + targetFlywheelVelocity + feedforward);
                    io.runFlywheel(targetFlywheelVelocity, feedforward, false);

                    if (inputs.velocityRadPerSec >= targetFlywheelVelocity) {
                        inputs.velocityRadPerSec = targetFlywheelVelocity;
                       flywheelState = FlywheelState.RUNNING;
                    }
                }

                case RUNNING -> {
                    if (inputs.velocityRadPerSec >= targetFlywheelVelocity) {
                        inputs.velocityRadPerSec = targetFlywheelVelocity;
                        io.runOpenLoop(feedforward, true);
                    } else {
                        flywheelState = FlywheelState.RAMP_UP;
                    }
                }
            }
        }

        Logger.recordOutput("Flywheel/State", flywheelState.toString());
        Logger.recordOutput("Flywheel/TargetVelocity", targetFlywheelVelocity);
    }

    // velocity sampling:

    private void sampleCurrentVelocity() {
        sampledVelocity = inputs.velocityRadPerSec;
        targetFlywheelVelocity = sampledVelocity;
        Logger.recordOutput("Flywheel/Sampled Velocity", sampledVelocity);
    }

    public double getSampledVelocity() {
        return sampledVelocity;
    }

    public double getVelocity() {
        return inputs.velocityRadPerSec;
    }

    public boolean isAtTarget() {
        return Math.abs(inputs.velocityRadPerSec - targetFlywheelVelocity) <= flywheelVelocityTolerance;
    }

    // Command Factories

    public Command runAtSampleVelocity() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    sampleCurrentVelocity();
                    flywheelState = FlywheelState.RAMP_UP;
                }, this),
                Commands.waitUntil(this::isAtTarget),
                Commands.runOnce(() -> flywheelState = FlywheelState.RUNNING, this)
                        .withName("Flywheel.runAtSampledVelocity")
        );
    }

//    public Command runAtVelocity(double velocityRadPerSec) {
//        return Commands.runOnce(() -> {
//                    targetFlywheelVelocity = velocityRadPerSec;
//                    flywheelState = FlywheelState.RAMP_UP;
//                }, this)
//                .withName("Flywheel.runAtVelocity(" + velocityRadPerSec + ")");
//    }

    public void runAtVelocity(double velocityRadPerSec) {
        System.out.println("Reached the RunAtVelocity Function");
        targetFlywheelVelocity = velocityRadPerSec;
        flywheelState = FlywheelState.RAMP_UP;
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> {
            flywheelState = FlywheelState.IDLE;
            io.stopFlywheel();
        }, this).withName("Flywheel.stop");
    }
}
