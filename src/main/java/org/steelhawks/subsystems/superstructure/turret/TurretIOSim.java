package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.steelhawks.Constants;

public class TurretIOSim implements TurretIO {

    private final PIDController mController;
    private boolean pidEnabled = false;
    private double desiredPosition;
    private double ff;

    // volts = current * resistance + backEMF
    // backEMF = omega / kV

    private final double MAX_VOLTS = 12.0;

    private final DCMotorSim mMotor;

    public TurretIOSim() {
        mController = new PIDController(
            Turret.kP.get(),
            Turret.kI.get(),
            Turret.kD.get());
        mMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                0.001,
                Turret.reduction),
            DCMotor.getKrakenX60Foc(1));
    }

    public double currentToVolts(double current) {
        DCMotor motor = DCMotor.getKrakenX60Foc(1);
        double resistance = motor.rOhms;
        double omega = mMotor.getAngularVelocityRadPerSec();
        double backEMF = omega / motor.KvRadPerSecPerVolt;
        double volts = current * resistance + backEMF;

        return volts;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        mMotor.update(Constants.UPDATE_LOOP_DT);

        inputs.connected = true;
        inputs.positionRad = new Rotation2d(mMotor.getAngularPosition());
        inputs.velocityRadPerSec = new Rotation2d(mMotor.getAngularVelocityRadPerSec());
        inputs.appliedVolts = mMotor.getInputVoltage();
        inputs.currentAmps = mMotor.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = mMotor.getTorqueNewtonMeters() / DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
        inputs.tempCelsius = mMotor.getCurrentDrawAmps() * 0.1;

        if (pidEnabled) {
            double pid = mController.calculate(inputs.positionRad.getRadians(), desiredPosition);
            mMotor.setInputVoltage(currentToVolts(pid + ff));
        }
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        TurretIO.super.setBrakeMode(enabled);
    }

    @Override
    public void runPivot(double setpoint, double feedforward) {
        desiredPosition = setpoint;
        ff = feedforward;
        pidEnabled = true;
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        pidEnabled = false;
        mMotor.setInputVoltage(
            isTorqueCurrent
            ? currentToVolts(output)
            : output);
    }

    @Override
    public void runPercentOutput(double output) {
        pidEnabled = false;
        mMotor.setInputVoltage(output * MAX_VOLTS);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        mController.setPID(kp, ki, kd);
    }

    @Override
    public void setPosition(double position) {
        mMotor.setAngle(position);
    }

    @Override
    public void stop() {
        mMotor.setInputVoltage(0);
        mMotor.setAngularVelocity(0.0);
        pidEnabled = false;
    }
}
