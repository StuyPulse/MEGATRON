package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.vision.TheiaCamera;
import com.stuypulse.robot.util.vision.VisionData;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class TheiaTagVision extends AprilTagVision {

    private final TheiaCamera[] cameras;
    private final ArrayList<VisionData> outputs;

    private final FieldObject2d robot;

    protected TheiaTagVision() {
        this.cameras = new TheiaCamera[Cameras.APRILTAG_CAMERAS.length];
        for (int i = 0; i < Cameras.APRILTAG_CAMERAS.length; i++) {
            cameras[i] = new TheiaCamera(Cameras.APRILTAG_CAMERAS[i]);
        }

        outputs = new ArrayList<VisionData>();

        robot = Odometry.getInstance().getField().getObject("Vision Pose");
    }

    /**
     * Returns the outputs of the vision system.
     *
     * @return the outputs of the vision system
     */
    @Override
    public ArrayList<VisionData> getOutputs() {
        return outputs;
    }

    /**
     * Sets the tag layout of the vision system.
     *
     * @param ids the tag IDs
     */
    @Override
    public void setTagWhitelist(int... ids) {
        for (TheiaCamera camera : cameras) {
            camera.setTagLayout(ids);
        }
    }

    @Override
    public void setCameraEnabled(String name, boolean enabled) {
        for (TheiaCamera camera : cameras) {
            if (camera.getName().equals(name))
                camera.setEnabled(enabled);
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        outputs.clear();
        
        for (TheiaCamera camera : cameras) {
            SmartDashboard.putBoolean(camera.getName() + "/Has Data", camera.getVisionData().isPresent());

            camera.getVisionData().ifPresent(
                (VisionData data) -> {
                    outputs.add(data);
                    updateTelemetry("Vision/" + camera.getName(), data);
                });
        }

        SmartDashboard.putBoolean("Vision/Has Any Data", outputs.size() > 0);
    }

    private void updateTelemetry(String prefix, VisionData data) {
        SmartDashboard.putNumber(prefix + "/Pose X", data.getPose().getX());
        SmartDashboard.putNumber(prefix + "/Pose Y", data.getPose().getY());
        SmartDashboard.putNumber(prefix + "/Pose Z", data.getPose().getZ());

        SmartDashboard.putNumber(prefix + "/Distance to Tag", data.getDistanceToPrimaryTag());

        SmartDashboard.putNumber(prefix + "/Pose Rotation", Units.radiansToDegrees(data.getPose().getRotation().getAngle()));
        SmartDashboard.putNumber(prefix + "/Timestamp", data.getTimestamp());

        robot.setPose(data.getPose().toPose2d());
    }
}
