package org.steelhawks.subsystems.superstructure.turret;

public class TurretIOSim implements TurretIO {

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = true;
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        TurretIO.super.setBrakeMode(enabled);
    }

    @Override
    public void runPivot(double setpoint, double feedforward) {
        TurretIO.super.runPivot(setpoint, feedforward);
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        TurretIO.super.runOpenLoop(output, isTorqueCurrent);
    }

    @Override
    public void runPercentOutput(double output) {
        TurretIO.super.runPercentOutput(output);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        TurretIO.super.setPID(kp, ki, kd);
    }

    @Override
    public void setPosition(double position) {
        TurretIO.super.setPosition(position);
    }

    @Override
    public void stop() {
        TurretIO.super.stop();
    }
}
