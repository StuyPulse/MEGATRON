package com.stuypulse.robot.util.vision;

import com.stuypulse.robot.constants.Cameras.CameraConfig;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera extends PhotonCamera{

    private final Pose3d cameraLocation;
    private final PhotonPoseEstimator poseEstimator;

    private final SmartBoolean enabled;

    public PhotonVisionCamera(String name, Pose3d cameraLocation, String ip, int port) {
        super(name);
        this.cameraLocation = cameraLocation;

        poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.AVERAGE_BEST_TARGETS, cameraLocation.minus(new Pose3d()));

        enabled = new SmartBoolean(name + "/Enabled", true);
        
        PortForwarder.add(port, "10.6.94." + ip, 5802);
    }

    public PhotonVisionCamera(CameraConfig config) {
        this(config.getName(), config.getLocation(), config.getIP(), config.getForwardedPort());
    }

    public boolean hasData() {
        return super.getLatestResult().hasTargets();
    }

    /**
     * Returns the pose of the robot relative to the field.
     *
     * @return the pose of the robot relative to the field
     */
    private Pose3d getRobotPose() {
        PhotonTrackedTarget target = getLatestResult().getBestTarget();
        Transform3d camToTargetTrans = target.getBestCameraToTarget();
        Pose3d camPose = Field.getTag(target.getFiducialId()).getLocation().transformBy(camToTargetTrans.inverse());
        return camPose.transformBy(new Transform3d(cameraLocation.getTranslation(), cameraLocation.getRotation()).inverse());
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
            if (!data.isValidData()) return Optional.empty();
        }

        return Optional.empty();
    }
}
