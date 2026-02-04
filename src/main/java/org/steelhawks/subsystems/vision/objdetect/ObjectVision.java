package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.FieldConstants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.vision.VisionConstants;
import org.steelhawks.util.LoggedTunableNumber;
import org.steelhawks.util.VirtualSubsystem;

import java.util.*;
import java.util.stream.Collectors;

public class ObjectVision extends VirtualSubsystem {

    private static final double coralOverlap = 0.1; // meters
    private static final double coralMaxAge = 5.0; // seconds

    private static final LoggedTunableNumber maxArea =
        new LoggedTunableNumber("ObjectVision/MaxArea", 20.0);
    private static final LoggedTunableNumber confidenceThreshold =
        new LoggedTunableNumber("ObjectVision/ConfidenceThreshold", 0.3);
    private static final LoggedTunableNumber maxDetectableObjects =
        new LoggedTunableNumber("ObjectVision/MaxDetectableObjects", 5);

    private final ObjectVisionIO[] io;
    private final ObjectVisionIOInputsAutoLogged[] inputs;

    record CoralPose(
        Translation2d translation, double timestamp
    ) {}

    private final FieldObject2d coralObjects = FieldConstants.FIELD_2D.getObject("Corals");
    private final ArrayList<ObjectVisionIO.ObjectObservation> allObservations = new ArrayList<>();
    private final LinkedList<CoralPose> coralPoses = new LinkedList<>();

    public ObjectVision() {
        this.io = VisionConstants.getObjIO();
        this.inputs = new ObjectVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new ObjectVisionIOInputsAutoLogged();
        }
    }

    private void addCoralObservationToPose(ObjectVisionIO.ObjectObservation observation) {
        double now = Timer.getFPGATimestamp();
        Optional<Pose2d> oldWheelOdomPose = RobotContainer.s_Swerve.getPoseAtTime(observation.timestamp());
        if (Constants.loggedValue("ObjectProcessing/WheelOdomEmpty", oldWheelOdomPose.isEmpty())) {
            return;
        }
        var estimatedPose = RobotContainer.s_Swerve.getPose();
        var wheelOdometryPose = RobotContainer.s_Swerve.getWheelOdomPose();
        Pose2d fieldToRobot =
            estimatedPose.transformBy(new Transform2d(wheelOdometryPose, oldWheelOdomPose.get()));
        Transform3d robotToCamera =
            Objects.requireNonNull(VisionConstants.getObjDetectConfig())[observation.camIndex()].robotToCamera();

        // in angle coordinates
        double[] txCorners = observation.info().tx();  // degrees
        double[] tyCorners = observation.info().ty();  // degrees

        Constants.loggedValue("ObjectProcessing/TxCorners_degrees", txCorners);
        Constants.loggedValue("ObjectProcessing/TyCorners_degrees", tyCorners);

        // bounding box center in deg
        double minX = Math.min(Math.min(txCorners[0], txCorners[1]),
            Math.min(txCorners[2], txCorners[3]));
        double maxX = Math.max(Math.max(txCorners[0], txCorners[1]),
            Math.max(txCorners[2], txCorners[3]));
        double minY = Math.min(Math.min(tyCorners[0], tyCorners[1]),
            Math.min(tyCorners[2], tyCorners[3]));
        double maxY = Math.max(Math.max(tyCorners[0], tyCorners[1]),
            Math.max(tyCorners[2], tyCorners[3]));

        // in degrees
        double tx = (minX + maxX) / 2.0;
        double ty = (minY + maxY) / 2.0;

        Constants.loggedValue("ObjectProcessing/tx_degrees", tx);
        Constants.loggedValue("ObjectProcessing/ty_degrees", ty);

        // trig
        double cameraHeight = robotToCamera.getZ();
        double cameraPitch = robotToCamera.getRotation().getY(); // radians
        Constants.loggedValue("ObjectProcessing/cameraPitch_radians", cameraPitch);
        double tyRadians = Math.toRadians(ty);
        double verticalAngleFromHorizontal = -cameraPitch - tyRadians;
        Constants.loggedValue("ObjectProcessing/AngleFromHorizontal_radians", verticalAngleFromHorizontal);
        Constants.loggedValue("ObjectProcessing/AngleFromHorizontal_degrees", Math.toDegrees(verticalAngleFromHorizontal));
        if (Constants.loggedValue("ObjectProcessing/VerticalAngleError", verticalAngleFromHorizontal <= 0)) {
            return;
        }
        double targetHeight = 0.0;
        double forwardDistance = (cameraHeight - targetHeight) / Math.tan(verticalAngleFromHorizontal);
        double txRadians = Math.toRadians(tx);
        Translation2d cameraToCoralInCameraFrame =
            new Translation2d(
                forwardDistance * Math.cos(txRadians),
                forwardDistance * Math.sin(txRadians));
        Constants.loggedValue("ObjectProcessing/cameraToCoralInCameraFrame",
            new Pose2d(cameraToCoralInCameraFrame, new Rotation2d()));

        Transform2d robotToCamera2d = new Transform2d(
            robotToCamera.getTranslation().toTranslation2d(),
            robotToCamera.getRotation().toRotation2d());
        Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera2d);

        Constants.loggedValue("ObjectProcessing/fieldToRobot", fieldToRobot);
        Constants.loggedValue("ObjectProcessing/fieldToCamera", fieldToCamera);
        Constants.loggedValue("ObjectProcessing/forwardDistance", forwardDistance);

        Pose2d fieldToCoral = fieldToCamera
            .transformBy(new Transform2d(cameraToCoralInCameraFrame, new Rotation2d()));
        Constants.loggedValue("ObjectProcessing/fieldToCoral", fieldToCoral);
        CoralPose coralPose = new CoralPose(fieldToCoral.getTranslation(), observation.timestamp());
        coralPoses.removeIf(
            c -> c.translation.getDistance(fieldToCoral.getTranslation()) <= coralOverlap
                || now - c.timestamp > coralMaxAge);
        coralPoses.add(coralPose);

        // fifo
        while (coralPoses.size() > (int) maxDetectableObjects.get()) {
            CoralPose removed = coralPoses.removeFirst();
            Logger.recordOutput("ObjectProcessing/RemovedOldest",
                new Pose2d(removed.translation, new Rotation2d()));
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < inputs.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("ObjectVision/" + io[i].getName(), inputs[i]);
            allObservations.addAll(Arrays.asList(inputs[i].observations));
        }
        allObservations.stream()
            .filter(o -> o.confidence() >= confidenceThreshold.get())
            .sorted(Comparator.comparingDouble(ObjectVisionIO.ObjectObservation::timestamp))
            .forEach(this::addCoralObservationToPose);

        coralPoses.forEach(o ->
            Logger.recordOutput("CoralDetections/Detection", new Pose2d(o.translation, new Rotation2d())));
        coralObjects.setPoses(
            coralPoses.stream()
                .map(coral -> new Pose2d(coral.translation, new Rotation2d())) // zero rotation
                .collect(Collectors.toList()));
        allObservations.clear();
    }

    public void reset() {
        coralPoses.clear();
    }
}
