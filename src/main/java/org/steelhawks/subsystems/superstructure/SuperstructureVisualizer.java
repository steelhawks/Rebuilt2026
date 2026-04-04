package org.steelhawks.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.steelhawks.Constants;
import org.steelhawks.SubsystemConstants;

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

    // TODO: find a better way to get the appropriate constants into classes like this
    private static final Rotation2d hoodMaxAngle = SubsystemConstants.SimBot.HOOD.maxAngle();

    private static final double TURRET_X = Constants.RobotConstants.ROBOT_TO_TURRET.getX();
    private static final double TURRET_Y = Constants.RobotConstants.ROBOT_TO_TURRET.getY();

    private static final Translation3d HOOD_PIVOT = new Translation3d(
        Units.inchesToMeters(0.0),
        Units.inchesToMeters(-4.358),
        Units.inchesToMeters(-2.642)
    );

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
        double hood = Math.toDegrees(hoodRad.get());
        Pose2d robot = robotPoseSupplier.get();
        Logger.recordOutput("HoodVisualizer", hoodRad.get());

        // Mechanism2d
        turretLigament.setAngle(new Rotation2d(turret + Math.PI / 2));
        hoodLigament.setAngle(Rotation2d.fromDegrees(hoodRad.get()));
        Logger.recordOutput("Superstructure/Mechanism2d", mechanism);

        Translation2d turretOffset = new Translation2d(TURRET_X, TURRET_Y);
        Translation2d turretWorld = robot.getTranslation()
            .plus(turretOffset.rotateBy(robot.getRotation()));
        Logger.recordOutput("Superstructure/TurretPose", new Pose2d(
            turretWorld,
            robot.getRotation().plus(new Rotation2d(turret))));
        // model_0 = turret base yaw only
        // model_1 = hood, yaw + pitch
        Logger.recordOutput("Superstructure/ComponentPoses", new Pose3d[]{
            new Pose3d(
                Constants.RobotConstants.ROBOT_TO_TURRET.getTranslation(),
                new Rotation3d(0.0, 0.0, turret + Math.PI)),
            new Pose3d(
                Constants.RobotConstants.ROBOT_TO_TURRET.getTranslation().plus(HOOD_PIVOT),
                new Rotation3d(0.0, -(hoodMaxAngle.getRadians() - hood), turret + Math.PI))
        });
    }
}