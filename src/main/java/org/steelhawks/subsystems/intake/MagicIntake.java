package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.subsystems.superstructure.FuelStateTracker;
import org.steelhawks.util.BatteryUtil;
import org.steelhawks.util.LoggedTunableNumber;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

public class MagicIntake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SubsystemConstants.IntakeConstants constants;
    private final FuelStateTracker fuelStateTracker = new FuelStateTracker();

    // Goal — single double, no TrapezoidProfile state objects
    private double goalMeters = 0.0;
    private IntakeConstants.State desiredGoal = IntakeConstants.State.HOME;

    // Homing state
    private boolean isHomed = false;
    private boolean isZeroed = false;
    private boolean brakeModeEnabled = false;
    private boolean isRollersRunning = false;

    private final double homingVolts = 5.0;

    private final Debouncer homingDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    private final Debouncer twistingDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    // Tunable numbers — PID/FF updated live via LoggedTunableNumber
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;

    // Motion Magic profile constraints — tunable so you can adjust feel without redeploying
    private final LoggedTunableNumber maxVelocity;
    private final LoggedTunableNumber maxAccel;
    private final LoggedTunableNumber jerk;

    // Thresholds
    private final LoggedTunableNumber currentHomingThreshold;
    private final LoggedTunableNumber velocityStallingThreshold;
    private final LoggedTunableNumber positionTwistingThreshold;

    // Tuning overrides (lazy init — only created when tuning mode is on)
    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private final double FF_RAMP_RATE = 1.0;
    private final double DYNAMIC_STEP_VOLTAGE = 4.0;

    public MagicIntake(IntakeIO io, SubsystemConstants.IntakeConstants constants) {
        this.io = io;
        this.constants = constants;

        kP = new LoggedTunableNumber("Intake/kP", constants.kP());
        kI = new LoggedTunableNumber("Intake/kI", constants.kI());
        kD = new LoggedTunableNumber("Intake/kD", constants.kD());
        kS = new LoggedTunableNumber("Intake/kS", constants.kS());

        maxVelocity = new LoggedTunableNumber("Intake/MaxVelocityMetersPerSec", constants.cruiseVelocity());
        maxAccel = new LoggedTunableNumber("Intake/MaxAccelMetersPerSecSq", constants.maxAccelMetersPerSecSq());
        jerk = new LoggedTunableNumber("Intake/Jerk", constants.jerk());

        currentHomingThreshold = new LoggedTunableNumber("Intake/CurrentHomingThreshold", constants.currentHomingThreshold());
        velocityStallingThreshold = new LoggedTunableNumber("Intake/VelocityStallingThreshold", constants.velocityStallingThreshold());
        positionTwistingThreshold = new LoggedTunableNumber("Intake/PositionTwistingThreshold", constants.positionTwistingThreshold());
    }

    // ---- State queries ----

    @AutoLogOutput(key = "Intake/AtGoal")
    public boolean atGoal() {
        return Math.abs(inputs.leftPositionMeters - goalMeters) <= IntakeConstants.TOLERANCE;
    }

    public IntakeConstants.State getDesiredGoal() {
        return desiredGoal;
    }

    public double getPosition() {
        return inputs.leftPositionMeters;
    }

    public boolean isRollersRunning() {
        return isRollersRunning;
    }

    @AutoLogOutput(key = "Intake/IsStalling")
    private boolean isStalling() {
        return homingDebouncer.calculate(
                (Math.abs(inputs.leftTorqueCurrentAmps) > currentHomingThreshold.get()
                        && Math.abs(inputs.leftVelocityMetersPerSec) < velocityStallingThreshold.get())
                        || Math.abs(inputs.rightTorqueCurrentAmps) > currentHomingThreshold.get()
                        && Math.abs(inputs.rightVelocityMetersPerSec) < velocityStallingThreshold.get());
    }

    @AutoLogOutput(key = "Intake/IsTwisting")
    private boolean isTwisting() {
        return twistingDebouncer.calculate(
                Math.abs(inputs.leftPositionMeters - inputs.rightPositionMeters)
                        > positionTwistingThreshold.get()) && isHomed;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(enabled);
    }

    // ---- Periodic ----

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        fuelStateTracker.updateAtIntake((isRollersRunning && atGoal()) || (isRollersRunning));
        Logger.processInputs("Intake", inputs);
        BatteryUtil.recordCurrentUsage(
                "Intake",
                inputs.leftSupplyCurrentAmps
                        + inputs.rightSupplyCurrentAmps
                        + inputs.leftTorqueCurrentAmps
                        + inputs.rightTorqueCurrentAmps);

        // ---- Sim: skip homing, zero immediately ----
        if (Constants.getRobot().equals(Constants.RobotType.SIMBOT) && !isHomed && !isZeroed) {
            isHomed = true;
            isZeroed = true;
            io.setPosition(0);
            goalMeters = IntakeConstants.State.HOME.getPosition();
            Logger.recordOutput("Intake/IsHomed", true);
            Logger.recordOutput("Intake/Zeroed", true);
        }

        // ---- Homing sequence ----
        if (!isHomed && Toggles.Intake.isEnabled.get()) {
            io.runRackOpenLoop(homingVolts, false);
            isHomed = isStalling();
            Logger.recordOutput("Intake/IsHomed", isHomed);
        } else if (!isZeroed) {
            // First loop after homing — zero here, then let Motion Magic take over
            io.setPosition(IntakeConstants.State.INTAKE.getPosition());
            goalMeters = IntakeConstants.State.INTAKE.getPosition();
            isZeroed = true;
            Logger.recordOutput("Intake/Zeroed", true);
        }

        // ---- Mode transitions ----
        if (DriverStation.isDisabled()) {
            // Hold home position passively when disabled
            // Motion Magic will hold it — no need to repeatedly command
            if (Robot.isFirstRun()) setBrakeMode(false);
            return; // skip control logic entirely while disabled
        }

        if (DriverStation.isEnabled()) {
            setBrakeMode(true);
        }

        // ---- Live PID + profile updates via LoggedTunableNumber ----
        if (Toggles.tuningMode.get()) {
            LoggedTunableNumber.ifChanged(
                    hashCode(),
                    () -> io.setRackPID(kP.get(), kI.get(), kD.get()),
                    kP, kI, kD);

            LoggedTunableNumber.ifChanged(
                    hashCode() + 1,
                    () -> io.setMotionMagicConstraints(maxVelocity.get(), maxAccel.get(), jerk.get()),
                    maxVelocity, maxAccel, jerk);

            // Voltage override
            if (Toggles.Intake.toggleVoltageOverride.get()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Intake/TuningVolts", 0.0);
                }
                io.runRackOpenLoop(tuningVolts.get(), false);
                return;
            }

            // Current override
            if (Toggles.Intake.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Intake/TuningAmps", 0.0);
                }
                io.runRackOpenLoop(tuningAmps.get(), true);
                return;
            }
        }

        // ---- Run gate ----
        final boolean shouldRun =
                (isHomed && isZeroed || Constants.getRobot().equals(Constants.RobotType.SIMBOT))
                        && inputs.leftConnected
                        && inputs.rightConnected
                        && Toggles.Intake.isEnabled.get()
                        && !Toggles.Intake.toggleCurrentOverride.get()
                        && !Toggles.Intake.toggleVoltageOverride.get();

        Logger.recordOutput("Intake/ShouldRun", shouldRun);

        if (!shouldRun) return;

        // ---- Motion Magic position command ----
        // This is the entire control loop now — one line.
        // kS, kV, kA, kG, and the S-curve profile all run onboard at 1kHz.
        // The massive feedforward block that was here is gone.
        io.runRackPositionBoth(goalMeters, 0.0, 0.0);

        // ---- Logging ----
        Logger.recordOutput("Intake/GoalMeters", goalMeters);
        Logger.recordOutput("Intake/PositionErrorMeters", goalMeters - inputs.leftPositionMeters);
        Logger.recordOutput("Intake/AtGoal", atGoal());
        Logger.recordOutput("Intake/IsTwisting", isTwisting());
    }

    // ---- Goal setting ----

    public void setDesiredState(IntakeConstants.State state) {
        goalMeters = MathUtil.clamp(
                state.getPosition(),
                IntakeConstants.MIN_EXTENSION,
                IntakeConstants.MAX_EXTENSION_FROM_FRAME);
        desiredGoal = state;
        inputs.goal = goalMeters;
        // Motion Magic picks up the new goal on the next periodic() call
        // No profile recalculation needed — TalonFX generates a new S-curve onboard
    }

    public IntakeConstants.State getCurrentState() {
        return desiredGoal;
    }

    // ---- Commands ----

    public Command setDesiredStateCommand(IntakeConstants.State state) {
        return Commands.runOnce(() -> setDesiredState(state), this);
    }

    public Command moveToStateAndWait(IntakeConstants.State state) {
        return setDesiredStateCommand(state)
                .andThen(Commands.waitUntil(this::atGoal));
    }

    public Command slamOut() {
        return Commands.run(() -> io.runRackOpenLoop(5.0, false), this)
                .until(this::isStalling)
                .finallyDo(() -> io.setPosition(IntakeConstants.State.INTAKE.getPosition()));
    }

    public Command slamIn() {
        return Commands.run(() -> io.runRackOpenLoop(-5.0, false), this)
                .until(this::isStalling)
                .finallyDo(() -> io.setPosition(IntakeConstants.State.HOME.getPosition()));
    }

    public Command runIntake() {
        return Commands.run(() -> io.runIntake(constants.intakeSpeed()), this)
                .beforeStarting(() -> isRollersRunning = true)
                .finallyDo(() -> {
                    isRollersRunning = false;
                    io.stopIntake();
                });
    }

    public Command outtakeIntake() {
        return Commands.run(() -> io.runIntake(-constants.intakeSpeed()), this)
                .finallyDo(io::stopIntake);
    }

    public Command agitate() {
        return Commands.sequence(
                        setDesiredStateCommand(IntakeConstants.State.INTAKE),
                        Commands.waitUntil(this::atGoal),
                        setDesiredStateCommand(IntakeConstants.State.CENTER_OF_MOTION),
                        Commands.waitUntil(() -> atGoal() || isStalling()))
                .repeatedly()
                .finallyDo(() -> setDesiredState(IntakeConstants.State.HOME));
    }

    /**
     * Slow feed move — temporarily reduces Motion Magic cruise velocity on the TalonFX
     * to 30% for a careful retraction, then restores full speed.
     * Previously this rebuilt a TrapezoidProfile — now it just updates the onboard constraints.
     */
    public Command feed() {
        return Commands.sequence(
                Commands.runOnce(() ->
                        io.setMotionMagicConstraints(
                                constants.cruiseVelocity() * 0.3,
                                constants.maxAccelMetersPerSecSq() * 0.3,
                                constants.jerk()), this),
                setDesiredStateCommand(IntakeConstants.State.HOME),
                Commands.waitUntil(this::atGoal)
        ).finallyDo(() ->
                io.setMotionMagicConstraints(
                        constants.cruiseVelocity(),
                        constants.maxAccelMetersPerSecSq(),
                        constants.jerk()));
    }

    public Command zeroIntake() {
        return Commands.runOnce(() -> {
            isZeroed = false;
            isHomed = false;
        }, this);
    }



    public Command quasistaticCharacterization(boolean forward) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();
        double direction = forward ? 1.0 : -1.0;

        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                            timer.restart();
                        }
                ),
                Commands.run(
                                () -> {
                                    double voltage = direction * timer.get() * FF_RAMP_RATE;
                                    io.runAtSysIdVoltage(voltage);

                                    if (Math.abs(inputs.leftVelocityMetersPerSec) > 0.01) {
                                        velocitySamples.add(inputs.leftVelocityMetersPerSec);
                                        voltageSamples.add(inputs.leftAppliedVolts);
                                    }
                                }, this
                        )
                        .finallyDo(
                                () -> {
                                    io.stopRack();
                                    int n = velocitySamples.size();
                                    if (n < 2) {
                                        System.out.println("********** Intake Quasistatic: not enough data **********");
                                        return;
                                    }

                                    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Intake Quasistatic Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }
                        )
        );
    }


    public Command dynamicCharacterization(boolean forward) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> timeSamples = new LinkedList<>();
        Timer timer = new Timer();
        double stepVoltage = forward ? DYNAMIC_STEP_VOLTAGE : -DYNAMIC_STEP_VOLTAGE;

        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            timeSamples.clear();
                            timer.restart();
                        }
                ),
                Commands.run(() -> {
                            io.runAtSysIdVoltage(stepVoltage);
                            velocitySamples.add(inputs.leftVelocityMetersPerSec);
                            timeSamples.add(timer.get());
                        }, this)
                        .finallyDo(() -> {
                            io.stopRack();
                            int n = velocitySamples.size();
                            if (n < 2) {
                                System.out.println("********** Intake Dynamic: not enough data **********");
                                return;
                            }

                            double totalTime = timeSamples.get(n - 1) - timeSamples.get(0);
                            double totalVelocityChange = velocitySamples.get(n - 1) - velocitySamples.get(0);
                            double acceleration = totalVelocityChange / totalTime;

                            double kA = Math.abs(stepVoltage) / Math.abs(acceleration);
                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Intake Dynamic Characterization Results **********");
                            System.out.println("\tkA (approximate): " + formatter.format(kA));
                            System.out.println("\tAverage acceleration: " + formatter.format(acceleration) + " m/s²");

                        })
        );
    }
}