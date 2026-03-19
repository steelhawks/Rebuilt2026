package org.steelhawks.subsystems;

import au.grapplerobotics.CouldNotGetException;
import au.grapplerobotics.MitoCANdria;
import au.grapplerobotics.interfaces.MitoCANdriaInterface;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.util.VirtualSubsystem;

public class LoggedMitoCANdria extends VirtualSubsystem {
    MitoCANdria mito;
    public static final int MITOCANDRIA_ID = 0;
    public static final int ORANGE_PI_1_CHANNEL = MitoCANdriaInterface.MITOCANDRIA_CHANNEL_USB1;
    public static final int ORANGE_PI_2_CHANNEL = MitoCANdriaInterface.MITOCANDRIA_CHANNEL_USB2;
    public static final int LIMELIGHT_CHANNEL = MitoCANdriaInterface.MITOCANDRIA_CHANNEL_ADJ;
    private boolean last5VAEnabled = false;
    private boolean last5VBEnabled = false;
    private boolean lastAdjustableEnabled = false;

    public LoggedMitoCANdria(int roboRioCANId) {
        mito = new MitoCANdria(MITOCANDRIA_ID);
    }

    @Override
    public void periodic() {
        logChannel("OrangePi1", ORANGE_PI_1_CHANNEL);
        logChannel("OrangePi2", ORANGE_PI_2_CHANNEL);
        logChannel("Limelight", LIMELIGHT_CHANNEL);
    }

    private void logChannel(String channelName, int id) {
        try {
            mito.getChannelEnabled(id)
                .ifPresent((enabled) -> {
                    Logger.recordOutput("MitoCANDria/" + channelName + "Enabled", enabled == 1);
                });
            mito.getChannelCurrent(id)
                .ifPresent((current) -> Logger.recordOutput("MitoCANDria/" + channelName + "CurrentAmps", current));
        } catch (CouldNotGetException e) {
            DriverStation.reportWarning("Could not get status of MitoCANDria channel " + channelName + " at id " + id, false);
        }
    }
}
