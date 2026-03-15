package org.steelhawks.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.steelhawks.BuilderConstants;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.swerve.Swerve;

import static edu.wpi.first.units.Units.Inches;

public class IntakeIOSim implements IntakeIO {

    private final IntakeSimulation intakeSimulation;
    private final DCMotorSim left_extension_motor;
    private final DCMotorSim right_extension_motor;
    private final DCMotorSim roller_motor;

    private final double RADIUS = IntakeConstants.PINION_METERS_TO_RADIANS;

    private double feedforward = 0;
    private double goalPosition = 0;

    private final PIDController intakePIDController;
    private final IntakeVisualizer intakeVisualizer;

    private boolean enablePID;

    public IntakeIOSim(IntakeConstants intakeConstants) {
        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                Swerve.getDriveSimulation(),
                Inches.of(22),
                Inches.of(10.5),
                IntakeSimulation.IntakeSide.BACK,
                30
        );

        left_extension_motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        0.001,
                        IntakeConstants.EXTENSION_GEAR_RATIO.getAsDouble()
                ),
                DCMotor.getKrakenX44Foc(1)
        );

        right_extension_motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        0.001,
                        IntakeConstants.EXTENSION_GEAR_RATIO.getAsDouble()
                ),
                DCMotor.getKrakenX44Foc(1)
        );

        roller_motor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        0.001,
                        5.0
                ),
                DCMotor.getKrakenX44Foc(1)
        );

        intakePIDController = new PIDController(
                BuilderConstants.ExtensionPIDConstants.kP.getAsDouble(),
                BuilderConstants.ExtensionPIDConstants.kI.getAsDouble(),
                BuilderConstants.ExtensionPIDConstants.kD.getAsDouble()
        );

        intakeVisualizer = new IntakeVisualizer(
                () -> left_extension_motor.getAngularPositionRad() * IntakeConstants.PINION_METERS_TO_RADIANS,
                IntakeConstants.MAX_EXTENSION,
                -Math.PI + IntakeConstants.EXTENSION_ANGLE
        );
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        left_extension_motor.update(Constants.UPDATE_LOOP_DT);
        roller_motor.update(Constants.UPDATE_LOOP_DT);
        right_extension_motor.update(Constants.UPDATE_LOOP_DT);
        intakeVisualizer.update();

        inputs.leftConnected = true;
        inputs.leftExtensionPosition = left_extension_motor.getAngularPositionRad() * IntakeConstants.PINION_METERS_TO_RADIANS;
        inputs.leftExtensionVelocity = left_extension_motor.getAngularVelocityRadPerSec() * IntakeConstants.PINION_METERS_TO_RADIANS;
        inputs.leftExtensionAppliedVolts = left_extension_motor.getInputVoltage();
        inputs.leftExtensionCurrentAmps = left_extension_motor.getCurrentDrawAmps();
        inputs.leftExtensionCurrentAmps = left_extension_motor.getCurrentDrawAmps();
        inputs.leftExtensionTempCelsius = left_extension_motor.getCurrentDrawAmps() * 0.1;

        inputs.rightConnected = true;
        inputs.rightExtensionPosition = right_extension_motor.getAngularPositionRad() * IntakeConstants.PINION_METERS_TO_RADIANS;
        inputs.rightExtensionVelocity = right_extension_motor.getAngularVelocityRadPerSec() * IntakeConstants.PINION_METERS_TO_RADIANS;
        inputs.rightExtensionAppliedVolts = right_extension_motor.getInputVoltage();
        inputs.rightExtensionCurrentAmps = right_extension_motor.getCurrentDrawAmps();
        inputs.rightExtensionTorqueCurrent = right_extension_motor.getCurrentDrawAmps();
        inputs.rightExtensionTempCelsius = right_extension_motor.getCurrentDrawAmps() * 0.1;

        inputs.rollerConnected = true;
        inputs.rollerPosition = new Rotation2d(roller_motor.getAngularPositionRad());
        inputs.rollerVelocity = roller_motor.getAngularVelocityRadPerSec();
        inputs.rollerAppliedVolts = roller_motor.getInputVoltage();
        inputs.rollerCurrentAmps = roller_motor.getCurrentDrawAmps();
        inputs.rollerTorqueCurrent = roller_motor.getCurrentDrawAmps();
        inputs.rollerTempCelsius = roller_motor.getCurrentDrawAmps() * 0.1;

        if (enablePID) {
            double currentPosition = inputs.leftExtensionPosition;
            double pidOutput = intakePIDController.calculate(currentPosition, goalPosition);
            double commandedCurrent = pidOutput + feedforward;
            setMotorAmps(commandedCurrent);
        }
    }

    private void setMotorAmps(double commandedCurrent) {
        DCMotor motor = DCMotor.getKrakenX44Foc(2);
        double omega = left_extension_motor.getAngularVelocityRadPerSec();
        double backEmf = omega / motor.KvRadPerSecPerVolt;
        double volts = commandedCurrent * motor.rOhms + backEmf;
        left_extension_motor.setInputVoltage(volts);
        right_extension_motor.setInputVoltage(volts);
    }

    // rest of override functions
}