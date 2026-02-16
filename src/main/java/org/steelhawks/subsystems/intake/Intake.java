package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private IntakeState intakeState = IntakeState.IDLE;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


    enum IntakeState {
        IDLE,
        INTAKING,
        EJECTING,
        HOLDING,
        HOMING,
    }

    public Intake(IntakeIO io) {
        this.io = io;

        io.setExtensionBrakeMode(true);
        io.setRollerBrakeMode(false);

        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        IntakeConstants.MAX_VELOCITY_PER_SEC.getAsDouble(),
                        IntakeConstants.MAX_ACCEL_METERS_PER_SEC.getAsDouble()
                )
        );


        io.setExtensionBrakeMode(true);
        io.setRollerBrakeMode(false);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/State", intakeState.toString());
        Logger.processInputs("Intake", inputs);

        if (goal.position != setpoint.position) {
            updateMotionProfile();
        }

        Logger.recordOutput("Intake/Extension/Goal", goal.position);
        Logger.recordOutput("Intake/Extension/Setpoint", setpoint.position);
        Logger.recordOutput("Intake/Extension/SetpointVelocity", setpoint.velocity);
        Logger.processInputs("Intake", inputs);

    }



    private void updateMotionProfile() {
        setpoint = profile.calculate(
                0.02,
                setpoint,
                goal
        );

        double tuningVolts = calculateExtensionFeedForward(
                setpoint.velocity
        );

        io.setExtensionPosition(setpoint.position, tuningVolts);
    }

    private double calculateExtensionFeedForward(double velocity) {

        double frictionVolts = IntakeConstants.EXTENSION_POSITION_KS.getAsDouble() * Math.signum(velocity);
        double velocityVolts = IntakeConstants.EXTENSION_VELOCITY_KV.getAsDouble() * velocity;

        double accelerationVolts = 0.0;

        double gravityVolts = IntakeConstants.EXTENSION_POSITION_KG.getAsDouble();

        return frictionVolts + velocityVolts + accelerationVolts + gravityVolts;
    }

    private double calculateRollerFeedforward(double velocity) {
        double frictionVolts = IntakeConstants.ROLLER_KS.getAsDouble() * Math.signum(velocity);
        double velocityVolts = IntakeConstants.ROLLER_KV.getAsDouble() * velocity;

        return frictionVolts + velocityVolts;
    }


    public void intake() {
        // Use voltage control for simplicity
        io.setRollerVoltage(IntakeConstants.INTAKE_SPEED);
        intakeState = IntakeState.INTAKING;
    }

    /** Run the rollers to eject game pieces. */
    public void eject() {
        io.setRollerVoltage(IntakeConstants.EJECT_SPEED);
        intakeState = IntakeState.EJECTING;
    }


    /** Stop the rollers. */
    public void stopRollers() {
        io.stopRoller();
        if (intakeState == IntakeState.INTAKING || intakeState == IntakeState.EJECTING) {
            intakeState = IntakeState.IDLE;
        }
    }

    /** Run rollers at a specific voltage. */
    public void setRollerVoltage(double volts) {
        io.setRollerVoltage(volts);
    }

    /**
     * Run rollers at a specific velocity with feedforward (closed-loop).
     * This calculates the "tuning volts" automatically.
     */
    public void setRollerVelocity(double velocityRadPerSec) {
        double tuningVolts = calculateRollerFeedforward(velocityRadPerSec);
        io.setRollerVelocity(velocityRadPerSec, tuningVolts);
    }

    public void setExtensionGoal(double positionMeters) {

        // Clamp to safe limits
        positionMeters = Math.max(IntakeConstants.MIN_EXTENSION,
                Math.min(IntakeConstants.MAX_EXTENSION, positionMeters));

        // Set new goal - motion profile will handle smooth transition
        goal = new TrapezoidProfile.State(positionMeters, 0.0);

        // Reset profile to current state if starting from rest
        if (Math.abs(setpoint.velocity) < 0.01) {
            setpoint = new TrapezoidProfile.State(inputs.leftExtensionPosition, 0.0);
        }
    }

    public void retract() {
        setExtensionGoal(4);
    }

    /** Extend to the intake position. */
    public void extendToIntake() {
        setExtensionGoal(7);
    }

    /** Extend to the eject position. */
    public void extendToEject() {
        setExtensionGoal(8);
    }


}
