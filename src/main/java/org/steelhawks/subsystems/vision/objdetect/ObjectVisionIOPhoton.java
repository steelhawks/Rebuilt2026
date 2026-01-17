package org.steelhawks.subsystems.vision.objdetect;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.LinkedList;
import java.util.List;

public class ObjectVisionIOPhoton implements ObjectVisionIO {

    private final String name;
    private final int camIndex;

    private final PhotonCamera camera;

    public ObjectVisionIOPhoton(String name, int camIndex) {
        this.name = name;
        this.camIndex = camIndex;

        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(ObjectVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<ObjectObservation> observations = new LinkedList<>();
        List<PhotonPipelineResult> detections = camera.getAllUnreadResults();
        if (detections == null) {
            return;
        }

        for (PhotonPipelineResult detection : detections) {
            if (!detection.hasTargets()) {
                continue;
            }
            for (PhotonTrackedTarget target : detection.getTargets()) {
                observations.add(
                    new ObjectObservation(
                        camIndex,
                        new DetectionInfo(target),
                        target.getDetectedObjectConfidence(),
                        detection.getTimestampSeconds()
                    )
                );
            }

        }
        inputs.observations = observations.toArray(new ObjectObservation[0]);
    }

    @Override
    public String getName() {
        return name;
    }
}
