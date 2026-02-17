package org.steelhawks.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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

public class Intake extends SubsystemBase {

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private TrapezoidProfile profile;
    private final IntakeIO io;

    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;

    private double homingVolts = -2.0;

    private double testingVolts = 2.0;

    private IntakeConstants.State desiredGoal = IntakeConstants.State.HOME;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private boolean brakeModeEnabled = false;
    private boolean atGoal = false;
    private boolean isHomed = false;
    private boolean isZeroed = false;

    private final Debouncer homingDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);

    private static final LoggedTunableNumber currentHomingThres =
        new LoggedTunableNumber("Turret/CurrentHomingThreshold", 60.0);

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

    public double getPosition() {
        return inputs.leftPositionMeters;
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
        if (!isHomed) {
            io.runRackOpenLoop(testingVolts, false);
            isHomed = homingDebouncer.calculate(inputs.leftCurrentAmps > currentHomingThres.getAsDouble());
            Logger.recordOutput("Intake/IsHomed", isHomed);
        } else {
            if (!isZeroed) {
                io.setPosition(0.0);
                io.stopRack();
                isZeroed = true;
                Logger.recordOutput("Intake/Zeroed", true);
            }
        }
        final boolean shouldRun =
            DriverStation.isEnabled()
                && (inputs.leftConnected && inputs.rightConnected)
                && Toggles.Intake.isEnabled.get()
                && !Toggles.Intake.toggleCurrentOverride.get()
                && !Toggles.Intake.toggleVoltageOverride.get()
                && (getPosition() >= IntakeConstants.MIN_EXTENSION
                    && getPosition() <= IntakeConstants.MAX_EXTENSION);

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
            LoggedTunableNumber.ifChanged(this.hashCode(), () -> {
                io.setRackPID(
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
            if (setpoint.position < IntakeConstants.MIN_EXTENSION
                || setpoint.position > IntakeConstants.MAX_EXTENSION
            ) {
                setpoint =
                    new TrapezoidProfile.State(
                        MathUtil.clamp(
                            setpoint.position,
                            IntakeConstants.MIN_EXTENSION,
                            IntakeConstants.MAX_EXTENSION),
                        0.0);
            }
            atGoal = Math.abs(getPosition() - goal.position) <= IntakeConstants.TOLERANCE;
            if (atGoal) {
                io.stopRack();
            } else {
                // drivetrain accel
                //RAHMAN IS BEST
                double rawAccelY = RobotContainer.s_Swerve.getRobotRelativeYAccelGs();
                double pitchRadians = RobotContainer.s_Swerve.getPitch().getRadians();
                double drivetrainAccelG = rawAccelY - Math.sin(pitchRadians);
                double drivetrainAccel = drivetrainAccelG * 9.81;
                // physics based ff
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
                double staticFriction = IntakeConstants.kS.get() * Math.signum(setpoint.velocity);
                io.runRackPosition(
                    setpoint.position,
                    staticFriction);
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
            IntakeConstants.MAX_EXTENSION);
        goal = new TrapezoidProfile.State(inputs.goal, 0.0);
        desiredGoal = state;
    }

    public Command setDesiredStateCommand(IntakeConstants.State state) {
        return Commands.runOnce(() -> setDesiredState(state), this);
    }

    public Command runIntake() {
        return Commands.run(
            () -> {
                io.runIntake(IntakeConstants.INTAKE_SPEED);
            }, this).finallyDo(io::stopIntake);
    }

    public Command outtakeIntake() {
        return Commands.run(
            () -> io.runIntake(-IntakeConstants.INTAKE_SPEED), this).finallyDo(io::stopIntake);
    }
}

