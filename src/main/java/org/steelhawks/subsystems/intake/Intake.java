package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.*;
import org.steelhawks.util.BatteryUtil;
import org.steelhawks.util.LoggedTunableNumber;

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
    private final double homingVolts = 5.0;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;
    private boolean isRollersRunning = false;

    private final Debouncer homingDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
    private final Debouncer twistingDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private static LoggedTunableNumber currentHomingThreshold;
    private static LoggedTunableNumber velocityStallingThreshold;
    private static LoggedTunableNumber positionTwistingThreshold;
    private static LoggedTunableNumber kP;
    private static LoggedTunableNumber kI;
    private static LoggedTunableNumber kD;
    private static LoggedTunableNumber kS;
    private static LoggedTunableNumber crossCouple;
    private static LoggedTunableNumber MAX_VELOCITY_METERS_PER_SEC;
    private static LoggedTunableNumber MAX_ACCEL_METERS_PER_SEC_SQ;

    SubsystemConstants.IntakeConstants constants;

    public Intake(IntakeIO io, SubsystemConstants.IntakeConstants constants) {
        this.io = io;
        this.constants = constants;
        currentHomingThreshold = new LoggedTunableNumber("Intake/CurrentHomingThreshold", constants.currentHomingThreshold());
        velocityStallingThreshold = new LoggedTunableNumber("Intake/VelocityStallingThreshold", constants.velocityStallingThreshold());
        positionTwistingThreshold = new LoggedTunableNumber("Intake/PositionTwistingThreshold", constants.positionTwistingThreshold());
        kS = new LoggedTunableNumber("Intake/kS", constants.kS());
        kP = new LoggedTunableNumber("Intake/kP", constants.kP());
        kI = new LoggedTunableNumber("Intake/kI", constants.kI());
        kD = new LoggedTunableNumber("Intake/kD", constants.kD());
        crossCouple = new LoggedTunableNumber("Intake/CrossCouple", 0.3);
        MAX_ACCEL_METERS_PER_SEC_SQ = new LoggedTunableNumber("Intake/MaxAccelMetersPerSecSq", constants.maxAccelMetersPerSecSq());
        MAX_VELOCITY_METERS_PER_SEC = new LoggedTunableNumber("Intake/MaxVelocityMetersPerSec", constants.maxVelocityMetersPerSec());
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    MAX_VELOCITY_METERS_PER_SEC.get(),
                    MAX_ACCEL_METERS_PER_SEC_SQ.get()));
    }

    @AutoLogOutput(key = "Intake/AtGoal")
    public boolean atGoal() {
        return atGoal;
    }

    public IntakeConstants.State getDesiredGoal() {
        return desiredGoal;
    }

    public boolean atHome() {
        return atGoal && desiredGoal == IntakeConstants.State.HOME;
    }

    public double getPosition() {
        return inputs.leftPositionMeters;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    @AutoLogOutput(key = "Intake/IsStalling")
    private boolean isStalling() {
        return homingDebouncer.calculate(
            (Math.abs(inputs.leftTorqueCurrentAmps) > currentHomingThreshold.getAsDouble()
                && Math.abs(inputs.leftVelocityMetersPerSec) < velocityStallingThreshold.getAsDouble())
                || Math.abs(inputs.rightTorqueCurrentAmps) > currentHomingThreshold.getAsDouble()
                && Math.abs(inputs.rightVelocityMetersPerSec) < velocityStallingThreshold.getAsDouble());
    }

    @AutoLogOutput(key = "Intake/IsTwisting")
    private boolean isTwisting() {
        return twistingDebouncer.calculate(
            Math.abs(inputs.leftPositionMeters - inputs.rightPositionMeters) > positionTwistingThreshold.getAsDouble()) && isHomed;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        BatteryUtil.recordCurrentUsage(
            "Intake",
            inputs.leftSupplyCurrentAmps + inputs.rightSupplyCurrentAmps + inputs.leftTorqueCurrentAmps + inputs.rightTorqueCurrentAmps
        );

        if (Constants.getRobot().equals(Constants.RobotType.SIMBOT) && !isHomed && !isZeroed) {
            isHomed = true;
            io.setPosition(0);
            isZeroed = true;
            goal = new TrapezoidProfile.State(IntakeConstants.State.HOME.getPosition(), 0.0);
            setpoint = new TrapezoidProfile.State(IntakeConstants.State.HOME.getPosition(), 0.0);
            Logger.recordOutput("Intake/IsHomed", true);
            Logger.recordOutput("Intake/Zeroed", true);
        }

        if (!isHomed && Toggles.Intake.isEnabled.get()) {
            io.runRackOpenLoop(homingVolts, false);
            isHomed = isStalling();
            Logger.recordOutput("Intake/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(IntakeConstants.State.INTAKE.getPosition());
                io.stopRack();
                isZeroed = true;
                goal = new TrapezoidProfile.State(IntakeConstants.State.INTAKE.getPosition(), 0.0);
                setpoint = new TrapezoidProfile.State(IntakeConstants.State.INTAKE.getPosition(), 0.0);
                Logger.recordOutput("Intake/Zeroed", true);
            }
        }

        final boolean shouldRun =
            DriverStation.isEnabled()
                && ((isHomed && isZeroed) || Constants.getRobot().equals(Constants.RobotType.SIMBOT))
                && (inputs.leftConnected && inputs.rightConnected)
                && Toggles.Intake.isEnabled.get()
                && !Toggles.Intake.toggleCurrentOverride.get()
                && !Toggles.Intake.toggleVoltageOverride.get();
//                && (getPosition() >= IntakeConstants.MIN_EXTENSION
//                && getPosition() <= IntakeConstants.MAX_EXTENSION_FROM_FRAME + 0.05);
        Logger.recordOutput("Intake/ShouldRun", shouldRun);

        if (DriverStation.isDisabled()) {
            setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
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
                io.runRackOpenLoop(tuningVolts.get(), false);
            }
            if (Toggles.Intake.toggleCurrentOverride.get()) {
                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Intake/TuningAmps", 0.0);
                }
                io.runRackOpenLoop(tuningAmps.get(), true);
            }
            LoggedTunableNumber.ifChanged(this.hashCode(), () ->
                    io.setRackPID(kP.get(), kI.get(), kD.get()),
                kP, kI, kD);
            if (MAX_VELOCITY_METERS_PER_SEC.hasChanged(hashCode())
                || MAX_ACCEL_METERS_PER_SEC_SQ.hasChanged(hashCode())) {
                profile =
                    new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                            MAX_VELOCITY_METERS_PER_SEC.get(),
                            MAX_ACCEL_METERS_PER_SEC_SQ.get()));
            }
        }

        if (shouldRun) {
//            if (RobotContainer.s_Swerve.collisionDetected()) {
//                setDesiredState(IntakeConstants.State.RETRACTED);
//            }

            double previousVelocity = setpoint.velocity;
            setpoint = profile.calculate(Constants.UPDATE_LOOP_DT, setpoint, goal);
            if (setpoint.position <= IntakeConstants.MIN_EXTENSION
                || setpoint.position >= IntakeConstants.MAX_EXTENSION_FROM_FRAME) {
                setpoint =
                    new TrapezoidProfile.State(
                        MathUtil.clamp(
                            setpoint.position,
                            IntakeConstants.MIN_EXTENSION,
                            IntakeConstants.MAX_EXTENSION_FROM_FRAME),
                        0.0);
            }

            atGoal = Math.abs(getPosition() - goal.position) <= IntakeConstants.TOLERANCE;

            if (atGoal) {
                io.stopRack();
            } else {
                double rawAccelY = RobotContainer.s_Swerve.getRobotRelativeYAccelGs();
                double pitchRadians = RobotContainer.s_Swerve.getPitch().getRadians();
                double drivetrainAccelG = rawAccelY - Math.sin(pitchRadians);
                double drivetrainAccel = drivetrainAccelG * 9.81;

                double mass = IntakeConstants.MASS_KG;
                double rackAngle = IntakeConstants.RACK_ANGLE.getRadians();
                double pinionRadius = IntakeConstants.PINION_RADIUS;
                double gearRatio = IntakeConstants.REDUCTION;
                double frictionMultiplier = 1.3;
                double rackAccel = (setpoint.velocity - previousVelocity) / Constants.UPDATE_LOOP_DT;
                double forceGravity = mass * 9.81 * Math.sin(rackAngle);
                double forceRackAccel = mass * rackAccel;
                double forceDrivetrain = mass * drivetrainAccel * Math.cos(rackAngle);
                double totalForce = forceGravity + forceRackAccel + forceDrivetrain;
                double torqueAtPinion = totalForce * pinionRadius * frictionMultiplier;
                double motorTorque = torqueAtPinion / gearRatio;
                double kT = DCMotor.getKrakenX44Foc(2).KtNMPerAmp;
                double feedforwardCurrent = motorTorque / kT;

                double staticFriction = kS.get() * Math.signum(setpoint.velocity);
                double positionError = inputs.leftPositionMeters - inputs.rightPositionMeters;

                double leftFF = staticFriction;
                double rightFF = staticFriction + 0.5;

                Logger.recordOutput("Intake/LeftFF", leftFF);
                Logger.recordOutput("Intake/RightFF", rightFF);
                Logger.recordOutput("Intake/PositionError", positionError);

                io.runRackPositionBoth(setpoint.position, leftFF, rightFF);
            }

            Logger.recordOutput("Intake/SetpointPosition", setpoint.position);
            Logger.recordOutput("Intake/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Intake/GoalPosition", goal.position);
            Logger.recordOutput("Intake/GoalVelocity", goal.velocity);
        } else {
            setpoint = new TrapezoidProfile.State(getPosition(), 0.0);
            Logger.recordOutput("Intake/SetpointPosition", 0.0);
            Logger.recordOutput("Intake/SetpointVelocity", 0.0);
            Logger.recordOutput("Intake/GoalPosition", 0.0);
            Logger.recordOutput("Intake/GoalVelocity", 0.0);
        }
    }

    public void setDesiredState(IntakeConstants.State state) {
        inputs.goal = MathUtil.clamp(
            state.getPosition(),
            IntakeConstants.MIN_EXTENSION,
            IntakeConstants.MAX_EXTENSION_FROM_FRAME);
        goal = new TrapezoidProfile.State(inputs.goal, 0.0);
        desiredGoal = state;
    }

    public Command setDesiredStateCommand(IntakeConstants.State state) {
        return Commands.runOnce(() -> setDesiredState(state));
    }

    public Command slamOut() {
        return Commands.run(
                () -> io.runRackOpenLoop(5.0, false))
            .until(this::isStalling)
            .finallyDo(() -> io.setPosition(IntakeConstants.State.INTAKE.getPosition()));
    }

    public Command slamIn() {
        return Commands.run(
                () -> io.runRackOpenLoop(-5.0, false)).until(this::isStalling)
            .finallyDo(() -> io.setPosition(IntakeConstants.State.HOME.getPosition()));
    }

    public boolean isRollersRunning() {
        return isRollersRunning;
    }

    public Command runIntake() {
        return Commands.run(
                () -> io.runIntake(constants.intakeSpeed()), this)
            .beforeStarting(() -> isRollersRunning = true)
            .finallyDo(() -> {
                isRollersRunning = false;
                io.stopIntake();
            });
    }

    public Command outtakeIntake() {
        return Commands.run(
            () -> io.runIntake(-constants.intakeSpeed()), this).finallyDo(io::stopIntake);
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

    public Command feed() {
        return Commands.runOnce(() ->
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SEC.get() * 0.3,
                MAX_ACCEL_METERS_PER_SEC_SQ.get() * 0.3)))
        .andThen(setDesiredStateCommand(IntakeConstants.State.HOME))
        .andThen(Commands.waitUntil(this::atGoal))
        .finallyDo(() ->
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    MAX_VELOCITY_METERS_PER_SEC.get(),
                    MAX_ACCEL_METERS_PER_SEC_SQ.get())));
    }

    public Command zeroIntake() {
        return Commands.runOnce(() -> {
            isZeroed = false;
            isHomed = false;
        });
    }
}