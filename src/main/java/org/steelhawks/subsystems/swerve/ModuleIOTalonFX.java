package org.steelhawks.subsystems.swerve;

import static org.steelhawks.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.steelhawks.Constants;
import org.steelhawks.RobotConfig;
import org.steelhawks.Toggles;
import org.steelhawks.generated.TunerConstants;
import org.steelhawks.util.PhoenixUtil;

import java.util.Queue;


public class ModuleIOTalonFX implements ModuleIO {

    private final SwerveModuleConstants<
        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        constants;

    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final MotionMagicVoltage positionVoltageRequestMotionMagic = new MotionMagicVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
        new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
        new VelocityTorqueCurrentFOC(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // drive motor inputs
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;
    private final StatusSignal<Temperature> driveTemp;

    // turn motor inputs
    private final StatusSignal<Boolean> turnMagnetBad;
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;
    private final StatusSignal<Temperature> turnTemp;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFX(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            constants,
        RobotConfig.CANBus bus
    ) {
        this.constants = constants;
        driveTalon = new TalonFX(constants.DriveMotorId, bus.bus);
        turnTalon = new TalonFX(constants.SteerMotorId, bus.bus);
        cancoder = new CANcoder(constants.EncoderId, bus.bus);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted =
            constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource =
            switch (constants.FeedbackSource) {
                case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
                case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
                case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
                case FusedCANdiPWM1 -> FeedbackSensorSourceValue.FusedCANdiPWM1;
                case FusedCANdiPWM2 -> FeedbackSensorSourceValue.FusedCANdiPWM2;
                case SyncCANdiPWM1 -> FeedbackSensorSourceValue.SyncCANdiPWM1;
                case SyncCANdiPWM2 -> FeedbackSensorSourceValue.SyncCANdiPWM2;
                case RemoteCANdiPWM1 -> FeedbackSensorSourceValue.RemoteCANdiPWM1;
                case RemoteCANdiPWM2 -> FeedbackSensorSourceValue.RemoteCANdiPWM2;
                case TalonFXS_PulseWidth -> null;
            };
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration =
            turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted =
            constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection =
            constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig));

        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();
        driveTemp = driveTalon.getDeviceTemp();

        // Create turn status signals
        turnMagnetBad = cancoder.getFault_BadMagnet();
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getSupplyCurrent();
        turnTemp = turnTalon.getDeviceTemp();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
            Swerve.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            driveTemp,
            turnMagnetBad,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent,
            turnTemp);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder);
        PhoenixUtil.registerSignals(
            bus.bus.isNetworkFD(),
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            driveTemp,
            turnMagnetBad,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent,
            turnTemp);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Check all signals
        boolean driveStatus =
            BaseStatusSignal.isAllGood(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
        boolean turnStatus =
            BaseStatusSignal.isAllGood(turnMagnetBad, turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
        boolean turnEncoderStatus = BaseStatusSignal.isAllGood(turnAbsolutePosition);

        // drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus);
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
        inputs.driveTempCelsius = driveTemp.getValueAsDouble();

        // turn inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus);
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus);
        inputs.turnMagnetGood = !turnMagnetBad.getValue();
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
        inputs.turnTempCelsius = turnTemp.getValueAsDouble();

        // odometry inputs
        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map(Rotation2d::fromRotations)
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
                case Voltage -> voltageRequest.withOutput(output);
                case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
                case Voltage -> voltageRequest.withOutput(output);
                case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
            });
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
        driveTalon.setControl(
            switch (constants.DriveMotorClosedLoopOutput) {
                case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
                case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
            });
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnTalon.setControl(
            switch (constants.SteerMotorClosedLoopOutput) {
                case Voltage -> Toggles.motionMagicEnabled.get() ?
                    positionVoltageRequestMotionMagic.withPosition(rotation.getRotations()) :
                        positionVoltageRequest.withPosition(rotation.getRotations());
                case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
                    rotation.getRotations());
            });
    }

    @Override
    public void setDrivePID(double drivekP, double drivekI, double drivekD) {
        constants.DriveMotorGains.kP = drivekP;
        constants.DriveMotorGains.kI = drivekI;
        constants.DriveMotorGains.kD = drivekD;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(constants.DriveMotorGains));
    }

    @Override
    public void setSteerPID(double steerkP, double steerkI, double steerkD) {
        constants.SteerMotorGains.kP = steerkP;
        constants.SteerMotorGains.kI = steerkI;
        constants.SteerMotorGains.kD = steerkD;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(constants.SteerMotorGains));
    }
}
