package org.steelhawks.pi.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.steelhawks.pi.PiVisionConstants;

/**
 * Consumes local PhotonVision over NT (photonlib) and decodes each frame into a
 * robot-pose observation - multi-tag PnP when available, single-tag otherwise -
 * mirroring the old VisionIOPhoton. Frames are dropped until NT time-sync
 * converges (so timestamps can be put on the RIO clock).
 */
public class PhotonVisionIOReal implements PhotonVisionIO {

    private final PhotonCamera[] cameras;
    private final Transform3d[] robotToCamera;

    public PhotonVisionIOReal() {
        PiVisionConstants.Cam[] cfg = PiVisionConstants.CAMERAS;
        cameras = new PhotonCamera[cfg.length];
        robotToCamera = new Transform3d[cfg.length];
        for (int i = 0; i < cfg.length; i++) {
            cameras[i] = new PhotonCamera(cfg[i].name());
            robotToCamera[i] = cfg[i].robotToCamera();
        }
    }

    @Override
    public boolean allConnected() {
        for (PhotonCamera c : cameras) {
            if (!c.isConnected()) return false;
        }
        return true;
    }

    @Override
    public List<CameraObservation> poll() {
        List<CameraObservation> out = new ArrayList<>();
        for (int i = 0; i < cameras.length; i++) {
            for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
                OptionalDouble rioTime = TimeSync.toRioTime(result.getTimestampSeconds());
                if (rioTime.isEmpty() || !result.hasTargets()) continue;
                decode(i, result, rioTime.getAsDouble()).ifPresent(out::add);
            }
        }
        return out;
    }

    private Optional<CameraObservation> decode(int cam, PhotonPipelineResult result, double t) {
        Transform3d camToRobot = robotToCamera[cam].inverse();

        Optional<MultiTargetPNPResult> multi = result.getMultiTagResult();
        if (multi.isPresent()) {
            MultiTargetPNPResult m = multi.get();
            Pose3d robotPose =
                new Pose3d().plus(m.estimatedPose.best).plus(camToRobot);
            double avgDist = averageTagDistance(result.getTargets());
            int[] ids = new int[m.fiducialIDsUsed.size()];
            for (int k = 0; k < ids.length; k++) ids[k] = m.fiducialIDsUsed.get(k);
            return Optional.of(new CameraObservation(
                cam, t, robotPose, m.estimatedPose.ambiguity, ids.length, avgDist, ids));
        }

        // Single-tag fallback.
        PhotonTrackedTarget target = result.getBestTarget();
        if (target == null) return Optional.empty();
        Optional<Pose3d> tagPose = PiVisionConstants.APRIL_TAG_LAYOUT.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) return Optional.empty();
        Transform3d camToTag = target.getBestCameraToTarget();
        Pose3d robotPose = tagPose.get().plus(camToTag.inverse()).plus(camToRobot);
        double dist = camToTag.getTranslation().getNorm();
        return Optional.of(new CameraObservation(
            cam, t, robotPose, target.getPoseAmbiguity(), 1, dist,
            new int[] {target.getFiducialId()}));
    }

    private static double averageTagDistance(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget t : targets) {
            sum += t.getBestCameraToTarget().getTranslation().getNorm();
        }
        return sum / targets.size();
    }
}
