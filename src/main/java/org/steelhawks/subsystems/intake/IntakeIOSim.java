package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.steelhawks.Constants;
import org.steelhawks.SubsystemConstants;
import org.steelhawks.subsystems.swerve.Swerve;

import static edu.wpi.first.units.Units.Inches;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;
    private final DCMotorSim leftMotorSim;
    private final DCMotorSim rightMotorSim;
    private final DCMotorSim intakeMotorSim;

    private static final double RADIUS = IntakeConstants.METERS_PER_RADIAN;
    private double feedforwardCurrent = 0;
    private double goalPosition = 0;

    private final PIDController rackPidController;
    private final IntakeVisualizer visualizer;
    private boolean enablePid;

    public IntakeIOSim(SubsystemConstants.IntakeConstants constants) {
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            Swerve.getDriveSimulation(),
            Inches.of(22),
            Inches.of(10.5),
            IntakeSimulation.IntakeSide.BACK,
            30
        );

        leftMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                0.001,
                IntakeConstants.REDUCTION
            ),
            DCMotor.getKrakenX44Foc(1)
        );

        rightMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                0.001,
                IntakeConstants.REDUCTION
            ),
            DCMotor.getKrakenX44Foc(1)
        );

        intakeMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44Foc(1),
                0.001,
                5.0
            ),
            DCMotor.getKrakenX44Foc(1)
        );

        rackPidController = new PIDController(
            constants.kP(),
            constants.kI(),
            constants.kD()
        );
        visualizer = new IntakeVisualizer(
            () -> leftMotorSim.getAngularPositionRad() * IntakeConstants.METERS_PER_RADIAN,
            IntakeConstants.MAX_EXTENSION_FROM_FRAME,
            -Math.PI + IntakeConstants.RACK_ANGLE.getRadians());
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        leftMotorSim.update(Constants.UPDATE_LOOP_DT);
        intakeMotorSim.update(Constants.UPDATE_LOOP_DT);
        rightMotorSim.update(Constants.UPDATE_LOOP_DT);
        visualizer.update();

        inputs.leftConnected = true;
        inputs.leftPositionMeters = leftMotorSim.getAngularPositionRad() * IntakeConstants.METERS_PER_RADIAN;
        inputs.leftVelocityMetersPerSec = leftMotorSim.getAngularVelocityRadPerSec() * IntakeConstants.METERS_PER_RADIAN;
        inputs.leftAppliedVolts = leftMotorSim.getInputVoltage();
        inputs.leftSupplyCurrentAmps = leftMotorSim.getCurrentDrawAmps();
        inputs.leftTorqueCurrentAmps = leftMotorSim.getCurrentDrawAmps();
        inputs.leftTempCelsius = leftMotorSim.getCurrentDrawAmps() * 0.1;

        inputs.rightConnected = true;
        inputs.rightPositionMeters = rightMotorSim.getAngularPositionRad() * IntakeConstants.METERS_PER_RADIAN;
        inputs.rightVelocityMetersPerSec = rightMotorSim.getAngularVelocityRadPerSec() * IntakeConstants.METERS_PER_RADIAN;
        inputs.rightAppliedVolts = rightMotorSim.getInputVoltage();
        inputs.rightSupplyCurrentAmps = rightMotorSim.getCurrentDrawAmps();
        inputs.rightTorqueCurrentAmps = rightMotorSim.getCurrentDrawAmps();
        inputs.rightTempCelsius = rightMotorSim.getCurrentDrawAmps() * 0.1;

        inputs.intakeConnected = true;
        inputs.intakePositionRad = new Rotation2d(intakeMotorSim.getAngularPositionRad());
        inputs.intakeVelocityRadPerSec = intakeMotorSim.getAngularVelocityRadPerSec();
        inputs.intakeAppliedVolts = intakeMotorSim.getInputVoltage();
        inputs.intakeSupplyCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
        inputs.intakeTorqueCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
        inputs.intakeTempCelsius = intakeMotorSim.getCurrentDrawAmps() * 0.1;

        if (enablePid) {
            double currentPosition = inputs.leftPositionMeters;
            double pidOutput = rackPidController.calculate(currentPosition, goalPosition);
            double commandedCurrent = pidOutput + feedforwardCurrent;
            setMotorAmps(commandedCurrent);
        }
    }

    private void setMotorAmps(double commandedCurrent) {
        DCMotor motor = DCMotor.getKrakenX44Foc(2);
        double omega = leftMotorSim.getAngularVelocityRadPerSec();
        double backEmf = omega / motor.KvRadPerSecPerVolt;
        double volts = commandedCurrent * motor.rOhms + backEmf;
        leftMotorSim.setInputVoltage(volts);
        rightMotorSim.setInputVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        IntakeIO.super.setBrakeMode(enabled);
    }

    @Override
    public void runRackPosition(double positionMeters, double feedforward) {
        goalPosition = positionMeters;
        feedforwardCurrent = feedforward;
        enablePid = true;
    }

    @Override
    public void runRackOpenLoop(double output, boolean isTorqueCurrent) {
        if (isTorqueCurrent) {
            setMotorAmps(output);
        } else {
            leftMotorSim.setInputVoltage(output);
            rightMotorSim.setInputVoltage(output);
        }
    }

    @Override
    public void runRackPercentOut(double output) {
        leftMotorSim.setInputVoltage(output * 12);
        rightMotorSim.setInputVoltage(output * 12);
    }

    @Override
    public void setRackPID(double kP, double kI, double kD) {
        rackPidController.setPID(kP, kI, kD);
    }

    @Override
    public void setPosition(double meters) {
        double angle = meters / IntakeConstants.METERS_PER_RADIAN;
        leftMotorSim.setAngle(angle);
        rightMotorSim.setAngle(angle);
    }

    @Override
    public void runIntake(double output) {
        intakeMotorSim.setInputVoltage(output * 12);
        if (output >= 0) intakeSimulation.startIntake();
        intakeSimulation.addGamePiecesToIntake(2);
    }

    @Override
    public void stopRack() {
        leftMotorSim.setInputVoltage(0);
        rightMotorSim.setInputVoltage(0);
        enablePid = false;
    }

    @Override
    public void stopIntake() {
        intakeSimulation.stopIntake();
        intakeMotorSim.setInputVoltage(0);
    }
}
