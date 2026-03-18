package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.BuilderConstants;
import org.steelhawks.BuilderConstants.RollerPIDConstants;
import org.steelhawks.BuilderConstants.ExtensionPIDConstants;
import org.steelhawks.Toggles;
import org.steelhawks.util.LoggedTunableNumber;

import org.steelhawks.BuilderConstants.ExtensionPIDConstants.*;
import org.steelhawks.BuilderConstants.RollerPIDConstants.*;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private IntakeState intakeState = IntakeState.IDLE;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


    private LoggedTunableNumber tuningVolts;
    private LoggedTunableNumber tuningAmps;





    enum IntakeState {
        IDLE,
        INTAKING,
        EJECTING,
        HOLDING,
        HOMING,
    }

    public Intake(IntakeIO io) {
        this.io = io;

        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        IntakeConstants.MAX_VELOCITY_PER_SEC.getAsDouble(),
                        IntakeConstants.MAX_ACCEL_METERS_PER_SEC.getAsDouble()
                )
        );

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/State", intakeState.toString());

        if (goal.position != setpoint.position) {
            updateMotionProfile();
        }

        if (Toggles.tuningMode.getAsBoolean()) {
            // Fixed: unique hashCodes + varargs added
            LoggedTunableNumber.ifChanged(
                    hashCode(),
                    () -> io.setExtensionPID(
                            ExtensionPIDConstants.kP.get(),
                            ExtensionPIDConstants.kI.get(),
                            ExtensionPIDConstants.kD.get()
                    ),
                    ExtensionPIDConstants.kP,
                    ExtensionPIDConstants.kI,
                    ExtensionPIDConstants.kD
            );

            LoggedTunableNumber.ifChanged(
                    hashCode() + 1,
                    () -> io.setRollerPID(
                            RollerPIDConstants.kP.get(),
                            RollerPIDConstants.kI.get(),
                            RollerPIDConstants.kD.get()
                    ),
                    RollerPIDConstants.kP,
                    RollerPIDConstants.kI,
                    RollerPIDConstants.kD
            );

            // Log all PID values
            Logger.recordOutput("Intake/Extension/PID/kP", ExtensionPIDConstants.kP.get());
            Logger.recordOutput("Intake/Extension/PID/kI", ExtensionPIDConstants.kI.get());
            Logger.recordOutput("Intake/Extension/PID/kD", ExtensionPIDConstants.kD.get());

            Logger.recordOutput("Intake/Roller/PID/kP", RollerPIDConstants.kP.get());
            Logger.recordOutput("Intake/Roller/PID/kI", RollerPIDConstants.kI.get());
            Logger.recordOutput("Intake/Roller/PID/kD", RollerPIDConstants.kD.get());

            // Log all FF values — auto-update since calculateFF already calls .getAsDouble()
            Logger.recordOutput("Intake/Extension/FF/kS", IntakeConstants.EXTENSION_POSITION_KS.getAsDouble());
            Logger.recordOutput("Intake/Extension/FF/kV", IntakeConstants.EXTENSION_VELOCITY_KV.getAsDouble());
            Logger.recordOutput("Intake/Extension/FF/kG", IntakeConstants.EXTENSION_POSITION_KG.getAsDouble());

            Logger.recordOutput("Intake/Roller/FF/kS", RollerPIDConstants.kS.getAsDouble());
            Logger.recordOutput("Intake/Roller/FF/kV", RollerPIDConstants.kV.getAsDouble());

            if (Toggles.Intake.toggleVoltageOverride.getAsBoolean()) {
                if (tuningVolts == null) {
                    tuningVolts = new LoggedTunableNumber("Intake/TuningVolts", 0.0);
                }
                io.runExtensionOpenLoop(tuningVolts.getAsDouble(), false);

                if (tuningAmps == null) {
                    tuningAmps = new LoggedTunableNumber("Intake/TuningAmps", 0.0);
                }
            }
        }

        // These always log regardless of tuning mode
        Logger.recordOutput("Intake/Extension/Goal", goal.position);
        Logger.recordOutput("Intake/Extension/Setpoint", setpoint.position);
        Logger.recordOutput("Intake/Extension/SetpointVelocity", setpoint.velocity);
    }



    private void updateMotionProfile() {
        setpoint = profile.calculate(0.02, setpoint, goal);

        double ffOutput = calculateExtensionFeedForward(setpoint.velocity);

        // Log the computed FF output so you can see what's being sent
        Logger.recordOutput("Intake/Extension/FFOutput", ffOutput);

    }

    private double calculateExtensionFeedForward(double velocity) {
        double frictionVolts = IntakeConstants.EXTENSION_POSITION_KS.getAsDouble() * Math.signum(velocity);
        double velocityVolts = IntakeConstants.EXTENSION_VELOCITY_KV.getAsDouble() * velocity;
        double accelerationVolts = 0.0;
        double gravityVolts = IntakeConstants.EXTENSION_POSITION_KG.getAsDouble();

        // Log each component individually so you can see which term is dominating
        if (Toggles.tuningMode.getAsBoolean()) {
            Logger.recordOutput("Intake/Extension/FF/FrictionVolts", frictionVolts);
            Logger.recordOutput("Intake/Extension/FF/VelocityVolts", velocityVolts);
            Logger.recordOutput("Intake/Extension/FF/GravityVolts", gravityVolts);
        }

        return frictionVolts + velocityVolts + accelerationVolts + gravityVolts;
    }

    private double calculateRollerFeedforward(double velocity) {
        double frictionVolts = RollerPIDConstants.kS.getAsDouble() * Math.signum(velocity);
        double velocityVolts = RollerPIDConstants.kV.getAsDouble() * velocity;

        if (Toggles.tuningMode.getAsBoolean()) {
            Logger.recordOutput("Intake/Roller/FF/FrictionVolts", frictionVolts);
            Logger.recordOutput("Intake/Roller/FF/VelocityVolts", velocityVolts);
        }

        return frictionVolts + velocityVolts;
    }

    public void intake() {
        io.runIntake(IntakeConstants.INTAKE_SPEED);
        intakeState = IntakeState.INTAKING;
        Logger.recordOutput("Intake/Roller/AppliedVolts", IntakeConstants.INTAKE_SPEED);
    }

    public void eject() {
        io.runIntake(IntakeConstants.EJECT_SPEED);
        intakeState = IntakeState.EJECTING;
        Logger.recordOutput("Intake/Roller/AppliedVolts", IntakeConstants.EJECT_SPEED);
    }

    public void stopRollers() {
        io.stopIntake();
        Logger.recordOutput("Intake/Roller/AppliedVolts", 0.0);
        if (intakeState == IntakeState.INTAKING || intakeState == IntakeState.EJECTING) {
            intakeState = IntakeState.IDLE;
        }
    }

    public void setRollerVoltage(double volts) {
        io.runIntake(tuningVolts.getAsDouble());
        Logger.recordOutput("Intake/Roller/AppliedVolts", volts);
    }

    public void setRollerVelocity(double velocityRadPerSec) {
        double ffOutput = calculateRollerFeedforward(velocityRadPerSec);
        Logger.recordOutput("Intake/Roller/TargetVelocity", velocityRadPerSec);
        Logger.recordOutput("Intake/Roller/FFOutput", ffOutput);
        io.runIntake(tuningAmps.getAsDouble());
    }

    public void setExtensionGoal(double positionMeters) {
        positionMeters = Math.max(IntakeConstants.MIN_EXTENSION,
                Math.min(IntakeConstants.MAX_EXTENSION, positionMeters));
        goal = new TrapezoidProfile.State(positionMeters, 0.0);

        if (Math.abs(setpoint.velocity) < 0.01) {
            setpoint = new TrapezoidProfile.State(inputs.leftExtensionPosition, 0.0);
        }

        Logger.recordOutput("Intake/Extension/GoalPosition", positionMeters);
    }

// retract(), extendToIntake(), extendToEject() just call setExtensionGoal()
// so they're already covered — no changes needed

    public boolean atTargetPosition() {
        double positionError = Math.abs(goal.position - inputs.leftExtensionPosition);
        double velocityError = Math.abs(setpoint.velocity);
        boolean atTarget = positionError < IntakeConstants.POSITION_TOLERANCE
                && velocityError < IntakeConstants.MAX_VELOCITY_PER_SEC.getAsDouble();

        // Always useful to see these regardless of tuning mode
        Logger.recordOutput("Intake/Extension/PositionError", positionError);
        Logger.recordOutput("Intake/Extension/AtTarget", atTarget);

        return atTarget;
    }


}
