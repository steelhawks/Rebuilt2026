package org.steelhawks.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FusedSwerveDriveAlignment extends SwerveDriveAlignment {

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    public FusedSwerveDriveAlignment(Pose2d targetPose, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this(() -> targetPose, xSupplier, ySupplier);
    }

    public FusedSwerveDriveAlignment(Supplier<Pose2d> targetPose, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        super(targetPose);
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds =
            super.getOutput()
                .plus(
                    new ChassisSpeeds(
                        xSupplier.getAsDouble(),
                        ySupplier.getAsDouble(),
                        0));
        s_Swerve.runVelocity(speeds);
        super.updatePID();
        super.log();
    }
}
