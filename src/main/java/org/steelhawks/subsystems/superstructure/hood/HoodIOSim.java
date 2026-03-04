package org.steelhawks.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.steelhawks.Constants;
import org.steelhawks.SubsystemConstants;

public class HoodIOSim implements HoodIO {

    private final HoodVisualizer visualizer;
    private final PIDController controller;
    private final DCMotorSim hoodMotor;

    private boolean pidEnabled = false;
    private double desiredPositionRad;
    private double ff;
    private final double MAX_VOLTS = 12.0;

    public HoodIOSim(SubsystemConstants.HoodConstants constants) {
        controller = new PIDController(
            constants.kP(),
            constants.kI(),
            constants.kD());
        hoodMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                0.001,
                constants.reduction()),
            DCMotor.getKrakenX44Foc(1));
        visualizer = new HoodVisualizer(hoodMotor::getAngularPositionRad);
        hoodMotor.setAngle(constants.maxAngle().getRadians());
    }

    private double currentToVolts(double current) {
        DCMotor motor = DCMotor.getKrakenX44Foc(1);
        double omega = hoodMotor.getAngularVelocityRadPerSec();
        double backEMF = omega / motor.KvRadPerSecPerVolt;
        return current * motor.rOhms + backEMF;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        hoodMotor.update(Constants.UPDATE_LOOP_DT);

        inputs.motorConnected = true;
        inputs.motorPositionDeg = new Rotation2d(hoodMotor.getAngularPositionRad());
        inputs.motorVelocityDegPerSec = hoodMotor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = hoodMotor.getInputVoltage();
        inputs.currentAmps = hoodMotor.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = hoodMotor.getTorqueNewtonMeters() / DCMotor.getKrakenX44Foc(1).KtNMPerAmp;
        inputs.tempCelsius = hoodMotor.getCurrentDrawAmps() * 0.1;

        inputs.cancoderConnected = true;
        inputs.cancoderPositionDeg = inputs.motorPositionDeg;
        inputs.cancoderVelocityDegPerSec =  inputs.motorVelocityDegPerSec;
        inputs.cancoderAppliedVolts = Math.random() + MAX_VOLTS;

        if (pidEnabled) {
            double torqueCurrent = controller.calculate(inputs.motorPositionDeg.getRadians(), desiredPositionRad) + ff;
            hoodMotor.setInputVoltage(
                MathUtil.clamp(currentToVolts(torqueCurrent), -MAX_VOLTS, MAX_VOLTS));
        }
        visualizer.update();
    }

    @Override
    public void runHoodPosition(Rotation2d setpoint, double feedforward) {
        desiredPositionRad = setpoint.getRadians();
        ff = feedforward;
        pidEnabled = true;
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        pidEnabled = false;
        hoodMotor.setInputVoltage(
            isTorqueCurrent
                ? MathUtil.clamp(currentToVolts(output), -MAX_VOLTS, MAX_VOLTS)
                : MathUtil.clamp(output, -MAX_VOLTS, MAX_VOLTS));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    @Override
    public void stop() {
        pidEnabled = false;
        hoodMotor.setInputVoltage(0);
    }
}