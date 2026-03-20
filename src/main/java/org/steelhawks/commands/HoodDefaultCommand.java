package org.steelhawks.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotState;
import org.steelhawks.subsystems.superstructure.ShooterStructure;
import org.steelhawks.subsystems.superstructure.hood.Hood;
import org.steelhawks.util.AllianceFlip;

public class HoodDefaultCommand extends Command {

    private final Hood s_Hood;

    public HoodDefaultCommand(Hood s_Hood) {
        this.s_Hood = s_Hood;
        addRequirements(s_Hood);
    }

    @Override
    public void execute() {
        var hubCenter = AllianceFlip.apply(FieldConstants.Hub.HUB_CENTER_3D);
        if (RobotState.getInstance().getAimState().equals(RobotState.ShootingState.NOTHING)) {
            s_Hood.setDesiredPositionForced(Rotation2d.fromDegrees(80.0));
        } else if (RobotState.getInstance().getAimState().equals(RobotState.ShootingState.SHOOTING_STATIONARY)) {
            s_Hood.setDesiredPosition(Rotation2d.fromRadians(ShooterStructure.Static.calculateShot(hubCenter, hubCenter).hoodAngle()));
        }  else if (RobotState.getInstance().getAimState().equals(RobotState.ShootingState.SHOOTING_MOVING)) {
            s_Hood.setDesiredPosition(Rotation2d.fromRadians(ShooterStructure.Moving.calculateMovingShot(hubCenter, false).hoodAngle()));
        }
    }
}
