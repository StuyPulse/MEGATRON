package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSpeeds {

    private final Number shooterRPM; //the average RPM of left and right
    private final double shooterDifferential;

    private double leftRPM;
    private double rightRPM;

    public ShooterSpeeds() {
        this(0, 0);
    }

    public ShooterSpeeds(Number shooterRPM) {
        this(shooterRPM.doubleValue(), 0);
    }

    public ShooterSpeeds(Number shooterRPM, double shooterDifferential) {
        this.shooterRPM = shooterRPM;
        this.shooterDifferential = shooterDifferential;

        // update(Odometry.getInstance().getPose());
    }

    public ShooterSpeeds update(Pose2d robotPose) {
        // double higher = shooterRPM + shooterDifferential / 2.0;
        // double lower = shooterRPM - shooterDifferential / 2.0;

        // if (robotPose.getY() > Field.getAllianceSpeakerPose().getY()) {
        //     rightRPM = higher;
        //     leftRPM = lower;
        // } else {
        //     rightRPM = lower;
        //     leftRPM = higher;
        // }
        
        leftRPM = shooterRPM.doubleValue() + shooterDifferential / 2.0;
        rightRPM = shooterRPM.doubleValue() - shooterDifferential / 2.0;

        return this;
    }

    public double getLeftRPM() {
        return leftRPM;
    }

    public double getRightRPM() {
        return rightRPM;
    }
}
