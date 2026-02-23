package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.steelhawks.Constants;

import java.util.function.Supplier;

public class SuperstructureVisualizer {

    private static SuperstructureVisualizer instance;

    public static SuperstructureVisualizer getInstance() {
        if (instance == null) instance = new SuperstructureVisualizer();
        return instance;
    }

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d turretLigament;
    private final LoggedMechanismLigament2d hoodLigament;

    private Supplier<Double> turretRad = () -> 0.0;
    private Supplier<Double> hoodRad = () -> 0.0;
    private Supplier<Pose2d> robotPoseSupplier = Pose2d::new;

    // These are robot-relative offsets (relative to robot origin = floor center of robot)
    private static final double TURRET_X = Constants.RobotConstants.ROBOT_TO_TURRET.getX();
    private static final double TURRET_Y = Constants.RobotConstants.ROBOT_TO_TURRET.getY();
    private static final double TURRET_Z = Constants.RobotConstants.ROBOT_TO_TURRET.getZ();

    private SuperstructureVisualizer() {
        mechanism = new LoggedMechanism2d(3, 3);
        var root = mechanism.getRoot("SuperstructureRoot", 1.5, 1.5);
        turretLigament = new LoggedMechanismLigament2d(
            "Turret", 0.4, 90, 6, new Color8Bit(Color.kWhite));
        hoodLigament = new LoggedMechanismLigament2d(
            "Hood", 0.6, 0, 4, new Color8Bit(Color.kAqua));
        root.append(turretLigament);
        turretLigament.append(hoodLigament);
    }

    public void setTurretSupplier(Supplier<Double> turretRad) {
        this.turretRad = turretRad;
    }

    public void setHoodSupplier(Supplier<Double> hoodRad) {
        this.hoodRad = hoodRad;
    }

    public void setRobotPoseSupplier(Supplier<Pose2d> robotPose) {
        this.robotPoseSupplier = robotPose;
    }

    public void update() {
        double turret = turretRad.get();
        double hood = hoodRad.get();
        Pose2d robot = robotPoseSupplier.get();

        // Mechanism2d
        turretLigament.setAngle(new Rotation2d(turret + Math.PI / 2));
        hoodLigament.setAngle(new Rotation2d(-hood));
        Logger.recordOutput("Superstructure/Mechanism2d", mechanism);

        // 2D field turret pose
        Translation2d turretOffset = new Translation2d(TURRET_X, TURRET_Y);
        Translation2d turretWorld = robot.getTranslation()
            .plus(turretOffset.rotateBy(robot.getRotation()));
        Logger.recordOutput("Superstructure/TurretPose", new Pose2d(
            turretWorld,
            robot.getRotation().plus(new Rotation2d(turret))));

        // 3D component poses — robot-relative, AdvantageScope applies robot→field transform
        // model_0 = turret base yaw only
        // model_1 = hood, yaw + pitch
        Logger.recordOutput("Superstructure/ComponentPoses", new Pose3d[]{
            new Pose3d(
                new Translation3d(TURRET_X, TURRET_Y, TURRET_Z),
                new Rotation3d(0.0, 0.0, turret + Math.PI)
            ),
            new Pose3d(
                new Translation3d(TURRET_X, TURRET_Y, TURRET_Z),
                new Rotation3d(0.0, ShooterConstants.Hood.MAX_ANGLE.getRadians() - hood, turret + Math.PI)
            )
//            new Pose3d(
//                new Translation3d(TURRET_X, TURRET_Y, TURRET_Z),
//                new Rotation3d()
//            )
        });
    }
}