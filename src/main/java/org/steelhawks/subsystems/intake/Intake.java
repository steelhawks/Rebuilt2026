package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeState intakeState = IntakeState.IDLE;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final TrapezoidProfile profile;
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State();
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

    
}
