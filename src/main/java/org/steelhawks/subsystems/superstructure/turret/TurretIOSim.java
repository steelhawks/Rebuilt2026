package org.steelhawks.subsystems.superstructure.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Constants;

public class TurretIOSim implements TurretIO {
    private final PIDController pidController;
    private final DCMotorSim turretMotor;

    // private final TurretVisualizer turretVisualizer;

    private double feedfoward = 0.0;
    private double goalPosition = 0.0;

    private double MAX_VOLTS = 12.0;

    private boolean enablePID = false;

    public TurretIOSim(BuilderConstants.TurretConstants constants) {
        turretMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                0.001,
                constants.motorReduction()
        ),
                DCMotor.getKrakenX60Foc(1)
                );
        pidController = new PIDController(
                constants.kP(),
                constants.kI(),
                constants.kD()
        );

        /*   turretVisualizer = new TurretVisualizer(
            mMotor::getAngularPositionRad,
            () -> RobotState.getInstance().getEstimatedPose()); */

    }

    DCMotor motor = DCMotor.getKrakenX60Foc(1);

    public double currentToTorqueVolts(double current) {
        double torque = motor.KtNMPerAmp * current;

        double voltage = current * motor.rOhms
                + turretMotor.getAngularVelocityRadPerSec() / motor.KvRadPerSecPerVolt;
        return voltage;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {

        turretMotor.update(Constants.UPDATE_LOOP_DT);

        inputs.isConnected = true;
        inputs.position = new Rotation2d(turretMotor.getAngularPositionRad());
        inputs.velocityRadPerSec = new Rotation2d(turretMotor.getAngularVelocityRadPerSec());
        inputs.appliedVolts = turretMotor.getInputVoltage();
        inputs.currentAmps = turretMotor.getCurrentDrawAmps();
        inputs.torqueCurrent = turretMotor.getCurrentDrawAmps();
        inputs.statorCurrent = turretMotor.getCurrentDrawAmps();
        inputs.tempCelsius = turretMotor.getCurrentDrawAmps() * 0.1;

        if (enablePID) {
            double pid = pidController.calculate(inputs.position.getRadians(), goalPosition);
            turretMotor.setInputVoltage(currentToTorqueVolts(pid + feedfoward));
        }
    }

    @Override
    public void setTurretPosition(double position) {
        goalPosition = position;
        turretMotor.setAngle(position);
    }

    @Override
    public void runOpenLoop(double output, boolean isTorqueCurrent) {
        if (isTorqueCurrent) {
            turretMotor.setInputVoltage(currentToTorqueVolts(output));
        } else {
            turretMotor.setInputVoltage(output);
        }
    }

    @Override
    public void runTurret(double output) {
        turretMotor.setInputVoltage(output);
    }

    @Override
    public void stopTurret() {
        turretMotor.setInputVoltage(0.0);
    }

    @Override
    public void setTurretPID(double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
    }

}
