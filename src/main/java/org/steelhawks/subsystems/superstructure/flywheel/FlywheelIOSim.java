package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.steelhawks.Constants;
import org.steelhawks.SubsystemConstants;

public class FlywheelIOSim implements FlywheelIO {
    private DCMotorSim leftMotorSim;
    private DCMotorSim rightMotorSim;
    private PIDController velocityController;

    private double velocitySetpoint = 0;
    private boolean enablePid = false;
    private boolean useTorqueCurrent = false;
    private double feedforward = 0;

    public FlywheelIOSim(SubsystemConstants.FlywheelConstants constants) {
        leftMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1),
                0.001,
                constants.reduction()
            ),
            DCMotor.getKrakenX44(1)
        );

        rightMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1),
                0.001,
                constants.reduction()
            ),
            DCMotor.getKrakenX44(1)
        );

        velocityController = new PIDController(
            constants.kP(),
            constants.kI(),
            constants.kD()
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leftMotorSim.update(Constants.UPDATE_LOOP_DT);
        rightMotorSim.update(Constants.UPDATE_LOOP_DT);

        inputs.leftConnected = true;
        inputs.leftPositionRad = leftMotorSim.getAngularPositionRad();
        inputs.leftVelocityRadPerSec = leftMotorSim.getAngularVelocityRadPerSec();
        inputs.leftAppliedVolts = leftMotorSim.getInputVoltage();
        inputs.leftSupplyCurrentAmps = leftMotorSim.getCurrentDrawAmps();
        inputs.leftTorqueCurrentAmps = leftMotorSim.getTorqueNewtonMeters() / DCMotor.getKrakenX44(1).KtNMPerAmp;
        inputs.leftTempCelsius = inputs.leftSupplyCurrentAmps * 0.1;

        inputs.rightConnected = true;
        inputs.rightPositionRad = rightMotorSim.getAngularPositionRad();
        inputs.rightVelocityRadPerSec = rightMotorSim.getAngularVelocityRadPerSec();
        inputs.rightAppliedVolts = rightMotorSim.getInputVoltage();
        inputs.rightSupplyCurrentAmps = rightMotorSim.getCurrentDrawAmps();
        inputs.rightTorqueCurrentAmps = rightMotorSim.getCurrentDrawAmps() / DCMotor.getKrakenX44(1).KtNMPerAmp;
        inputs.leftTempCelsius = inputs.rightSupplyCurrentAmps * 0.1;

        if (enablePid) {
            double volts = 0;

            if (useTorqueCurrent) {
                double pid = velocityController.calculate(inputs.leftVelocityRadPerSec, velocitySetpoint);
                double commandedCurrent = pid + feedforward;
                volts = currentToVolts(commandedCurrent);
            } else {
                double pid = velocityController.calculate(inputs.leftVelocityRadPerSec, velocitySetpoint);
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
