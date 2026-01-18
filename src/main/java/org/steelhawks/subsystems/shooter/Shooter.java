package org.steelhawks.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    private final Shoo inputs;

    public Shooter(ShooterIO io) {
        this.io = io;
        this.pidController = new PIDController(ShooterConstants.SHOOTER_KP.getAsDouble(), ShooterConstants.SHOOTER_KI.getAsDouble(), ShooterConstants.SHOOTER_KD.getAsDouble());
        this.feedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS.getAsDouble(), ShooterConstants.SHOOTER_KV.getAsDouble());
        this.inputs = new ShooterIOInputsAutoLogged();
    }

    public double getVelocity() {
        io.updateInputs(inputs);
        return inputs.velocityPerSecRad;

    }

    @Override
    public void periodic() {
        double currentVelocity = getVelocity();

        double pidOutput =  pidController.calculate(currentVelocity, ShooterConstants.SHOOTER_MOTOR_MAX_RPM);



    }

}
