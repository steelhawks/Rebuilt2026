package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

public class FlywheelIOSim implements FlywheelIO {
    private DCMotorSim leftMotorSim;
    private DCMotorSim rightMotorSim;
    private PIDController velocityController;

    private double velocitySetpoint = 0;
    private boolean enablePid = false;
    private boolean useTorqueCurrent = false;
    private double feedforward = 0;

    public FlywheelIOSim() {
        leftMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1),
                0.001,
                1.0 / 2.0
            ),
            DCMotor.getKrakenX44(1)
        );

        rightMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1),
                0.001,
                1.0 / 2.0
            ),
            DCMotor.getKrakenX44(1)
        );

        velocityController = new PIDController(
            Flywheel.kP.get(),
            Flywheel.kI.get(),
            Flywheel.kD.get()
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leftMotorSim.update(Constants.UPDATE_LOOP_DT);
        rightMotorSim.update(Constants.UPDATE_LOOP_DT);

        inputs.connected = true;
        inputs.positionRad = leftMotorSim.getAngularPositionRad();
        inputs.velocityRadPerSec = leftMotorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = leftMotorSim.getInputVoltage();
        inputs.currentAmps = leftMotorSim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = leftMotorSim.getTorqueNewtonMeters() / DCMotor.getKrakenX44(1).KtNMPerAmp;
        inputs.tempCelsius = inputs.currentAmps * 0.1;

        if (enablePid) {
            double volts = 0;

            if (useTorqueCurrent) {
                double pid = velocityController.calculate(inputs.velocityRadPerSec, velocitySetpoint);
                double commandedCurrent = pid + feedforward;
                volts = currentToVolts(commandedCurrent);
            } else {
                double pid = velocityController.calculate(inputs.velocityRadPerSec, velocitySetpoint);
                volts = pid + feedforward;
            }

            leftMotorSim.setInputVoltage(volts);
            rightMotorSim.setInputVoltage(volts);
        }
    }

    @Override
    public void runFlywheel(double setpoint, double feedforward, boolean isTorqueCurrent) {
        velocitySetpoint = setpoint;
        useTorqueCurrent = isTorqueCurrent;
        this.feedforward = feedforward;
        enablePid = true;
    }

    @Override
    public void runFlywheelOpenLoop(double output, boolean isTorqueCurrent) {
        enablePid = false;
        if (isTorqueCurrent) {
            double volts = currentToVolts(output);
            leftMotorSim.setInputVoltage(volts);
            rightMotorSim.setInputVoltage(volts);
        } else {
            leftMotorSim.setInputVoltage(output);
            rightMotorSim.setInputVoltage(output);
        }
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        velocityController.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        enablePid = false;
        leftMotorSim.setInputVoltage(0);
        rightMotorSim.setInputVoltage(0);
    }

    public double currentToVolts(double current) {
        DCMotor motor = DCMotor.getKrakenX60Foc(1);
        double resistance = motor.rOhms;
        double omega = leftMotorSim.getAngularVelocityRadPerSec();
        double backEMF = omega / motor.KvRadPerSecPerVolt;
        double volts = current * resistance + backEMF;

        return volts;
    }
}
