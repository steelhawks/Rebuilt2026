package org.steelhawks.pi;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Entry point for the Orange Pi vision/pose service. Runs a headless
 * AdvantageKit {@link org.littletonrobotics.junction.LoggedRobot} off-robot
 * (the Idun pattern), so the whole fusion pipeline is loggable and replayable.
 */
public final class PiMain {
    private PiMain() {}

    public static void main(String... args) {
        RobotBase.startRobot(PiRobot::new);
    }
}
