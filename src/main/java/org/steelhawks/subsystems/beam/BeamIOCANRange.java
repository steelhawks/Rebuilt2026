package org.steelhawks.subsystems.beam;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;
import org.steelhawks.util.PhoenixUtil;

import static org.steelhawks.util.PhoenixUtil.tryUntilOk;

public class BeamIOCANRange implements BeamIO {

    private final CANrange range;

    private final CANrangeConfiguration config;
    private final StatusSignal<Distance> distanceSignal;
    private final StatusSignal<Boolean> detectedSignal;

    public BeamIOCANRange(CANBus bus, BeamBreakConfig initConfig) {
        range = new CANrange(initConfig.id(), bus);

        config = new CANrangeConfiguration();
        config.ProximityParams.ProximityHysteresis = initConfig.proximityHysteresis();
        config.ProximityParams.ProximityThreshold = initConfig.proximityThreshold();
        config.ToFParams.UpdateMode = initConfig.mode().equals(UpdateMode.SHORT_RANGE)
            ? UpdateModeValue.ShortRangeUserFreq
            : UpdateModeValue.LongRangeUserFreq;
        config.ToFParams.UpdateFrequency = initConfig.frequency();
        tryUntilOk(5, () -> range.getConfigurator().apply(config));

        distanceSignal = range.getDistance();
        detectedSignal = range.getIsDetected();

        PhoenixUtil.registerSignals(
            bus, distanceSignal, detectedSignal);
    }

    @Override
    public void updateInputs(BeamIOInputs inputs) {
        inputs.connected = range.isConnected();
        inputs.distanceMeters = distanceSignal.getValueAsDouble();
        inputs.detected = detectedSignal.getValue();
    }
}
