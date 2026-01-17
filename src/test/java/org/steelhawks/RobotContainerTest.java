

package org.steelhawks;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

public class RobotContainerTest {

    @Test
    public void createRobotContainer() {
        try {
            new RobotContainer();
        } catch (Exception e) {
            e.printStackTrace();
            fail("RobotContainer creation failed");
        }
    }
}
