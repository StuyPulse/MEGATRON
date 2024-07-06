package com.stuypulse.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSpeeds {

    private final Number shooterRPM; //the average RPM of left and right shooters
    private final Number shooterDifferential;

    private double leftRPM;
    private double rightRPM;

    public ShooterSpeeds() {
        this(0, 0);
    }

    public ShooterSpeeds(Number shooterRPM) {
        this(shooterRPM, 0);
    }

    public ShooterSpeeds(Number shooterRPM, Number shooterDifferential) {
        this.shooterRPM = shooterRPM;
        this.shooterDifferential = shooterDifferential;
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
        
        leftRPM = shooterRPM.doubleValue() + shooterDifferential.doubleValue() / 2.0;
        rightRPM = shooterRPM.doubleValue() - shooterDifferential.doubleValue() / 2.0;

        return this;
    }

    public double getLeftRPM() {
        return leftRPM;
    }

    public double getRightRPM() {
        return rightRPM;
    }
}
