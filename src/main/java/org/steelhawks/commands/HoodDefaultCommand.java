package org.steelhawks.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotState;
import org.steelhawks.Toggles;
import org.steelhawks.subsystems.shooterSuperstructure.ShooterStructure;
import org.steelhawks.subsystems.shooterSuperstructure.hood.Hood;
import org.steelhawks.util.AllianceFlip;

public class HoodDefaultCommand extends Command {

    private final Hood s_Hood;

    public HoodDefaultCommand(Hood s_Hood) {
        this.s_Hood = s_Hood;
        addRequirements(s_Hood);
    }

    @Override
    public void execute() {
        if (Toggles.shooterTuningMode.get()) return;
        var sol = RobotState.getInstance().getMovingShotSolution();
        switch (RobotState.getInstance().getShootingState()) {
            case NOTHING -> s_Hood.setDesiredPositionForced(Rotation2d.fromDegrees(80.0));
            case SHOOTING_STATIONARY -> {
                var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
                s_Hood.setDesiredPosition(Rotation2d.fromRadians(
                    ShooterStructure.Static.calculateShot(hubCenter, hubCenter).hoodAngle()));
            }
            case SHOOTING_MOVING -> {
                if (RobotState.getInstance().getAimState().equals(RobotState.AimState.FERRY)) {
                    s_Hood.setDesiredPosition(Rotation2d.fromRadians(ShooterStructure.Static.calculateFerryShot(AllianceFlip.apply(
                        FieldConstants.getClosestPointOnLine(
                            FieldConstants.Ferrying.START_LINE,
                            FieldConstants.Ferrying.END_LINE))).hoodAngle()));
                    return;
                }
                if (sol != null) {
                    s_Hood.setDesiredPosition(Rotation2d.fromRadians(sol.hoodAngleRad()));
                }
            }
        }
    }
}
