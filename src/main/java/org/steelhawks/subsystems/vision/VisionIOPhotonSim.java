package org.steelhawks.subsystems.vision;

import static org.steelhawks.subsystems.vision.VisionConstants.APRIL_TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonSim extends VisionIOPhoton {
    private static VisionSystemSim visionSim;
    private static double lastUpdateTimestamp = Double.NEGATIVE_INFINITY;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(
        String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(APRIL_TAG_LAYOUT);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    /**
     * Called on the main thread once per loop before futures are submitted.
     * The timestamp guard ensures visionSim.update() runs exactly once per loop
     * even though Vision.periodic() calls this for every camera index.
     */
    @Override
    public void updateSim() {
        double now = Timer.getFPGATimestamp();
        if (now != lastUpdateTimestamp) {
            lastUpdateTimestamp = now;
            visionSim.update(poseSupplier.get());
        }
    }

    /**
     * Reads the camera's NT results. visionSim.update() has already run on the
     * main thread via updateSim(), so this is just a fast NT read — safe to run
     * in a background thread without any timeout risk.
     */
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        super.updateInputs(inputs);
    }
}
