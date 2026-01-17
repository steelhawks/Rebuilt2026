package org.steelhawks.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.RobotContainer;
import org.steelhawks.commands.align.SwerveDriveAlignment;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.vision.Vision;
import org.steelhawks.util.AllianceFlip;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Command factory class for commands that require multiple subsystems.
 */
public class SuperStructure {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final Vision s_Vision = RobotContainer.s_Vision;

    /**
     * Wheel odometry is pretty inaccurate, so to prevent misalignment, we run this command to back up until we see a tag to relocalize.
     * If a tag is already in view, this command does nothing and continues the composition it is run in.
     *
     * @return Backup command or Commands.none().
     */
    public static Command continueIfTagInView(Supplier<Pose2d> toPose) {
        final double BACKUP_TIMEOUT = 1.5;
        final double FINAL_ALIGN_TIMEOUT = 1.0;
        final int minTag = AllianceFlip.shouldFlip() ? 6 : 17;
        final int maxTag = minTag + 5;

        BooleanSupplier needsToGetBack = () -> {
            // 0 is left mount, 1 is right mount
            int leftId = s_Vision.getTargetId(0);
            int rightId = s_Vision.getTargetId(1);

            return (leftId == -1 || leftId < minTag || leftId > maxTag)
                && (rightId == -1 || rightId < minTag || rightId > maxTag);
        };
        Logger.recordOutput("Align/HasVisionOfTag", !needsToGetBack.getAsBoolean());
        if (!needsToGetBack.getAsBoolean()) {
            return Commands.none();
        }

        return Commands.run(
            () ->
                s_Swerve.runVelocity(
                    new ChassisSpeeds(
                        -0.3,
                        0.0,
                        0.0)),
                s_Swerve)
            .until(() -> !needsToGetBack.getAsBoolean())
            .withTimeout(BACKUP_TIMEOUT)
            .andThen(
                new SwerveDriveAlignment(toPose).withTimeout(FINAL_ALIGN_TIMEOUT));
    }
}
