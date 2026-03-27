package org.steelhawks.subsystems.superstructure.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Constants;

public class FlywheelIOSim implements FlywheelIO {

    private final DCMotorSim left_motor;
    private final DCMotorSim right_motor;;

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

    DCMotor motor = DCMotor.getKrakenX44Foc(2);

    private double getAppliedVolts(double velocity, double ffoutput) {
        // velocity is rad/s, so measure in rad/s too
        double velocityErrorRadPerSec = velocity
                - left_motor.getAngularVelocityRadPerSec(); // no conversion needed

        double totalCurrentAmps = BuilderConstants.OmegaBot.FLYWHEEL.kP() * velocityErrorRadPerSec + ffoutput;

        double motorResistanceOhms = motor.nominalVoltageVolts / motor.stallCurrentAmps;
        double backEMFVolts = left_motor.getAngularVelocityRadPerSec() / motor.KvRadPerSecPerVolt;

        return MathUtil.clamp(
                totalCurrentAmps * motorResistanceOhms + backEMFVolts,
                -12.0, 12.0);
    }

    public void currentToTorqueVolts(double current) {
        double torque = motor.KtNMPerAmp * current;

        double voltage = current * motor.rOhms
                + left_motor.getAngularVelocityRadPerSec() / motor.KvRadPerSecPerVolt;

        left_motor.setInputVoltage(voltage);
        right_motor.setInputVoltage(voltage);
    }

    @Override
    public void runFlywheel(double velocity, double ffoutput, boolean isTorqueCurrent) {
        if (isTorqueCurrent) {
            double appliedVolts = getAppliedVolts(velocity, ffoutput);
            left_motor.setInputVoltage(appliedVolts);
            right_motor.setInputVoltage(appliedVolts);
        } else {
            // Stay in rad/s — matches what Flywheel.java passes in
            double currentVelocityRadPerSec =
                    (left_motor.getAngularVelocityRadPerSec()
                            + right_motor.getAngularVelocityRadPerSec()) / 2.0;

            double velocityErrorRadPerSec = velocity - currentVelocityRadPerSec;

            double appliedVolts = MathUtil.clamp(
                    BuilderConstants.OmegaBot.FLYWHEEL.kP() * velocityErrorRadPerSec + ffoutput,
                    -12.0, 12.0
            );

            left_motor.setInputVoltage(appliedVolts);
            right_motor.setInputVoltage(appliedVolts);
            Logger.recordOutput("Flywheel/AppliedVoltsSim", appliedVolts);
        }

        left_motor.update(Constants.UPDATE_LOOP_DT);
        right_motor.update(Constants.UPDATE_LOOP_DT);
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        if (isTorqueCurrent) {
            currentToTorqueVolts(output);
        } else {
            left_motor.setInputVoltage(output);
            right_motor.setInputVoltage(output);
        }
        left_motor.update(Constants.UPDATE_LOOP_DT);
        right_motor.update(Constants.UPDATE_LOOP_DT);
    }

    @Override
    public void stopFlywheel() {
        left_motor.setInputVoltage(0.0);
        right_motor.setInputVoltage(0.0);
        left_motor.update(Constants.UPDATE_LOOP_DT);
        right_motor.update(Constants.UPDATE_LOOP_DT);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        // No update() call here anymore
        inputs.isConnected = true;
        inputs.velocityRadPerSec = right_motor.getAngularVelocityRadPerSec();
        inputs.appliedVolts = right_motor.getInputVoltage();
        inputs.currentAmps = right_motor.getCurrentDrawAmps();
        inputs.torqueCurrent = right_motor.getCurrentDrawAmps();
        inputs.statorCurrent = right_motor.getCurrentDrawAmps();
        inputs.tempCelsius = right_motor.getCurrentDrawAmps() * 0.1;
    }

}
