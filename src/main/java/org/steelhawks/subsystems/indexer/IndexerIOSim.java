package org.steelhawks.subsystems.indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.steelhawks.Constants;
import org.steelhawks.SubsystemConstants;

public class IndexerIOSim implements IndexerIO {
    private DCMotorSim indexerMotor1;
    private DCMotorSim indexerMotor2;
    private DCMotorSim feederMotor;


    public IndexerIOSim(SubsystemConstants.IndexerConstants c) {
        indexerMotor1 = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44(1),
                        0.001,
                        1.0
                ),
                DCMotor.getKrakenX44(1)
        );
        indexerMotor1 = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44(1),
                        0.001,
                        1.0
                ),
                DCMotor.getKrakenX44(1)
        );
        feederMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44(1),
                        0.001,
                        1.0
                ),
                DCMotor.getKrakenX44(1)
        );
    }

    @Override
    public void updateInputs(SpindexerIOInputs spindexerInputs, FeederIOInputs feederInputs) {
        indexerMotor1.update(Constants.UPDATE_LOOP_DT);
        indexerMotor2.update(Constants.UPDATE_LOOP_DT);
        feederMotor.update(Constants.UPDATE_LOOP_DT);

        spindexerInputs.motor1Connected = true;
        spindexerInputs.motor1PositionRad = indexerMotor1.getAngularPositionRad();
        spindexerInputs.motor1VelocityRadPerSec = indexerMotor1.getAngularVelocityRadPerSec();
        spindexerInputs.motor1AppliedVolts = indexerMotor1.getInputVoltage();
        spindexerInputs.motor1CurrentAmps = indexerMotor1.getCurrentDrawAmps();
        spindexerInputs.motor1TorqueCurrentAmps = indexerMotor1.getTorqueNewtonMeters() / DCMotor.getKrakenX44(1).KtNMPerAmp;
        spindexerInputs.motor1TempCelsius = spindexerInputs.motor1CurrentAmps * 0.1;

        spindexerInputs.motor2Connected = true;
        spindexerInputs.motor2PositionRad = indexerMotor2.getAngularPositionRad();
        spindexerInputs.motor2VelocityRadPerSec = indexerMotor2.getAngularVelocityRadPerSec();
        spindexerInputs.motor2AppliedVolts = indexerMotor2.getInputVoltage();
        spindexerInputs.motor2CurrentAmps = indexerMotor2.getCurrentDrawAmps();
        spindexerInputs.motor2TorqueCurrentAmps = indexerMotor2.getCurrentDrawAmps() /  DCMotor.getKrakenX44(1).KtNMPerAmp;
        spindexerInputs.motor2TempCelsius = spindexerInputs.motor2CurrentAmps * 0.1;

        feederInputs.connected = true;
        feederInputs.positionRad = feederMotor.getAngularPositionRad();
        feederInputs.velocityRadPerSec = feederMotor.getAngularVelocityRadPerSec();
        feederInputs.currentAmps = feederMotor.getInputVoltage();
        feederInputs.appliedVolts = feederMotor.getInputVoltage();
        feederInputs.torqueCurrentAmps = feederMotor.getCurrentDrawAmps() /  DCMotor.getKrakenX44(1).KtNMPerAmp;
        feederInputs.tempCelsius = feederMotor.getCurrentDrawAmps() * 0.1;

    }

    @Override
    public void runSpindexer(double output) {
        indexerMotor1.setInputVoltage(output);
        indexerMotor2.setInputVoltage(output);
    }

    @Override
    public void runFeeder(double output) {
        feederMotor.setInputVoltage(output);
    }

    @Override
    public void stopFeeder() {
        feederMotor.setInputVoltage(0);
    }

    @Override
    public void stopSpindexer() {
        indexerMotor1.setInputVoltage(0);
        indexerMotor2.setInputVoltage(0);
    }

}
