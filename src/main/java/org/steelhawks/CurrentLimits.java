package org.steelhawks;

public interface CurrentLimits {

    interface SupplyLimit {
        double flywheelCurrent = 60.0;
        double hoodCurrent = 20.0;
        double turretCurrent = 20.0;
        double intakePositionCurrent = 20.0;
        double intakeRollersCurrent = 40.0;
        double spindexerCurrent = 20.0;
        double feederCurrent = 40.0;

        boolean flywheelEnabled = true;
        boolean hoodEnabled = true;
        boolean turretEnabled = false;
        boolean intakePositionEnabled = true;
        boolean intakeRollersEnabled = false;
        boolean spindexerEnabled = true;
        boolean feederEnabled = true;
    }

    interface StatorLimit {
        double driveCurrent = 45.0;
        double turnCurrent = 25.0;
        double flywheelCurrent = 80.0;
        double hoodCurrent = 60.0;
        double turretCurrent = 40.0;
        double intakePositionCurrent = 40.0;
        double intakeRollersCurrent = 80.0;
        double spindexerCurrent = 40.0;
        double feederCurrent = 80.0;

        boolean flywheelEnabled = true;
        boolean hoodEnabled = true;
        boolean turretEnabled = false;
        boolean intakePositionEnabled = true;
        boolean intakeRollersEnabled = false;
        boolean spindexerEnabled = true;
        boolean feederEnabled = true;
    }
}
