package org.steelhawks;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.steelhawks.subsystems.led.LEDMatrix;
import org.steelhawks.subsystems.led.LEDStrip;

public class RobotContainerTest {

    @AfterEach
    public void resetRegistry() {
        // Free HAL-allocated LED ports so the next robot's construction can reuse them.
        // Without this, the second robot to allocate the same AddressableLED port throws
        // AllocationException and fails the test.
        Subsystems.stripIfPresent().ifPresent(LEDStrip::close);
        Subsystems.matrixIfPresent().ifPresent(LEDMatrix::close);
        Constants.overrideRobotForTest(null);
        Subsystems.install(null);
    }

    /**
     * The regression gate that would have caught the prior Alpha-boot NPE:
     * every RobotType must construct cleanly without exception. Bindings that
     * reference a subsystem the current robot doesn't have must be guarded
     * via Subsystems.xIfPresent() rather than failing at construction.
     */
    @ParameterizedTest
    @EnumSource(Constants.RobotType.class)
    public void createRobotContainerForEveryRobot(Constants.RobotType type) {
        Constants.overrideRobotForTest(type);
        Subsystems.install(null);
        try {
            new RobotContainer();
        } catch (Exception e) {
            e.printStackTrace();
            fail("RobotContainer creation failed for " + type + ": " + e);
        }
    }
}
