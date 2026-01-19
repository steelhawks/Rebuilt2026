package org.steelhawks.subsystems.TurretSuperStructure.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final PIDController pidController;
    private final SimpleMotorFeedforward feedforward;
    private final ShooterIOInputsAutoLogged inputs;

    public Shooter(ShooterIO io) {
        this.io = io;
        this.pidController = new PIDController(ShooterConstants.SHOOTER_KP.getAsDouble(), ShooterConstants.SHOOTER_KI.getAsDouble(), ShooterConstants.SHOOTER_KD.getAsDouble());
        this.feedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_KS.getAsDouble(), ShooterConstants.SHOOTER_KV.getAsDouble());
        this.inputs = new ShooterIOInputsAutoLogged();
    }

    public double getVelocity() {
        io.updateInputs(inputs);
        return inputs.velocityRadPerSec;

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        double currentVelocity = getVelocity();

        double pidOutput =  pidController.calculate(currentVelocity, ShooterConstants.SHOOTER_MOTOR_MAX_RPM);
        double ffOutput = feedforward.calculate(ShooterConstants.SHOOTER_MOTOR_MAX_RPM);

        double totalVoltage = pidOutput + ffOutput;

        io.runSpeed(totalVoltage);

    }

}
