package org.steelhawks.subsystems.Superstructure.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.intake.IntakeConstants;
import org.steelhawks.subsystems.intake.IntakeVisualizer;

public class FlywheelIOSim implements FlywheelIO {

    private final DCMotorSim left_motor;
    private final DCMotorSim right_motor;

    private double feedforward = 0.0;
    private double targetVelocity = 0.0;
    private double velocityVoltage = 0.0;
    private double velocityVoltageTorque = 0.0;

    private final PIDController pidController;

    public FlywheelIOSim(BuilderConstants.FlywheelConstants constants) {

        left_motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        0.001, constants.reduction()),
                DCMotor.getKrakenX44Foc(1));
        right_motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        0.001, constants.reduction()),
                DCMotor.getKrakenX44Foc(1)
        );

        pidController = new PIDController(constants.kP(), constants.kI(), constants.kD());
    }


    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        left_motor.update(Constants.UPDATE_LOOP_DT);
        right_motor.update(Constants.UPDATE_LOOP_DT);

        inputs.isConnected = true;
        inputs.position = right_motor.getAngularPositionRad() * IntakeConstants.PINION_METERS_TO_RADIANS;
        inputs.velocityRadPerSec = right_motor.getAngularVelocityRadPerSec() * IntakeConstants.PINION_METERS_TO_RADIANS;
        inputs.appliedVolts = right_motor.getInputVoltage();
        inputs.currentAmps = right_motor.getCurrentDrawAmps();
        inputs.torqueCurrent = right_motor.getCurrentDrawAmps();
        inputs.statorCurrent  = right_motor.getCurrentDrawAmps();
        inputs.tempCelsius = right_motor.getCurrentDrawAmps() * 0.1;

    }

    public void currentToTorqueVolts(double current) {
        DCMotor motor = DCMotor.getKrakenX44Foc(2);

        double torque = motor.KtNMPerAmp * current;

        double voltage = current * motor.rOhms
                + left_motor.getAngularVelocityRadPerSec() / motor.KvRadPerSecPerVolt;

        left_motor.setInputVoltage(voltage);
        right_motor.setInputVoltage(voltage);
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        if (isTorqueCurrent) {
            currentToTorqueVolts(output);
        } else {
            left_motor.setInputVoltage(output);
            right_motor.setInputVoltage(output);
        }
    }

    @Override
    public void runFlywheel(double velocity, double ffoutput, boolean isTorqueCurrent) {
        
    }


}
