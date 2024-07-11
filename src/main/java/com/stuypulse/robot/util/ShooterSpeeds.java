package com.stuypulse.robot.util;

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

    public double getLeftRPM() {
        return shooterRPM.doubleValue() + shooterDifferential.doubleValue() / 2.0;
    }

    public double getRightRPM() {
        return shooterRPM.doubleValue() - shooterDifferential.doubleValue() / 2.0;
    }
}
