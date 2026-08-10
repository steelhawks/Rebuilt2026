package org.steelhawks.pi;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Entry point for the Orange Pi vision/pose service. Runs a headless
 * AdvantageKit {@link org.littletonrobotics.junction.LoggedRobot} off-robot
 * (the Idun pattern), so the whole fusion pipeline is loggable and replayable.
 */
public final class PiMain {
    private PiMain() {}

    public static void main(String... args) {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.startClient4("poselink-pi");
        nt.setServer("10.26.1.2");
        RobotBase.startRobot(PiRobot::new);

        // startRobot returning means the loop ended normally a restart commanded
        // from the RIO. Exit explicitly so a lingering non-daemon NT thread cannot
        // keep the JVM alive: the systemd unit is Restart=always, so the process
        // dying IS the restart, and a process that merely stops looping would sit
        // there forever with no vision and no restart.
        System.exit(0);
    }
}
