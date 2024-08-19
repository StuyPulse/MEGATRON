package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Cameras.CameraConfig;
import com.stuypulse.stuylib.network.SmartBoolean;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera extends PhotonCamera{

    private final PhotonPoseEstimator poseEstimator;

    private final SmartBoolean enabled;

    public PhotonVisionCamera(String name, Pose3d cameraLocation) {
        super(name);

        poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.AVERAGE_BEST_TARGETS, cameraLocation.minus(new Pose3d()));

        enabled = new SmartBoolean(name + "/Enabled", true);
    }

    public PhotonVisionCamera(CameraConfig config) {
        this(config.getName(), config.getLocation());
    }

    public boolean hasData() {
        return getLatestResult().hasTargets();
    }

    /**
     * Returns the IDs of the tags detected.
     *
     * @return the IDs of the tags detected
     */
    private int[] getIDs() {
        List<PhotonTrackedTarget> targets = getLatestResult().getTargets();
        int[] ids = new int[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            ids[i] = targets.get(i).getFiducialId();
        }
        return ids;
    }

    public void setEnabled(boolean enabled) {
        this.enabled.set(enabled);
    }

    /**
     * Returns an Optional holding the vision data from the camera.
     *
     * @return the vision data from the camera in an Optional
     */
    public Optional<VisionData> getVisionData() {
        if (!hasData()) return Optional.empty();
        if (!enabled.get()) return Optional.empty();

        PhotonPipelineResult latestData = getLatestResult();
        Optional<EstimatedRobotPose> robotPose = poseEstimator.update(latestData);
        if (robotPose.isPresent()) {
            VisionData data = new VisionData(robotPose.get().estimatedPose, getIDs(), latestData.getTimestampSeconds(), latestData.getBestTarget().getArea());
            if (data.isValidData()) {
                return Optional.of(data);
            }
        }

        SmartDashboard.putNumber("Vision/" + getName() + "/pose-ambiguity", latestData.getBestTarget().getPoseAmbiguity());
        return Optional.empty();
    }
}
