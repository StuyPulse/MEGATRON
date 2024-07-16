package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    
    private static final Shooter instance;

    static {
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }
    
    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    public Shooter() {
        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", Settings.Shooter.SPEAKER.getLeftRPM());
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", Settings.Shooter.SPEAKER.getRightRPM());
    }

    public void setTargetSpeeds(ShooterSpeeds speeds) {
        this.leftTargetRPM.set(speeds.getLeftRPM());
        this.rightTargetRPM.set(speeds.getRightRPM());
    }

    public double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }

    public double getRightTargetRPM() {
        return rightTargetRPM.get();
    }

    public void stop() {
        stopFeeder();
        leftTargetRPM.set(0);
        rightTargetRPM.set(0);
    }

    public abstract boolean atTargetSpeeds();

    public abstract void runFeederForwards();
    public abstract void runFeederBackwards();
    public abstract void stopFeeder();
    
    public abstract boolean hasNote();

    public abstract boolean noteShot();
}
