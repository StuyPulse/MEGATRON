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
        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", 0);
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", 0);
    }

    public final double getLeftTargetRPM() {
        return leftTargetRPM.get();
    }

    public final double getRightTargetRPM() {
        return rightTargetRPM.get();
    }

    public final void setTargetSpeeds(ShooterSpeeds speeds) {
        this.leftTargetRPM.set(speeds.getLeftRPM());
        this.rightTargetRPM.set(speeds.getRightRPM());
    }

    public final boolean atTargetSpeeds() {
        return Math.abs(getLeftShooterRPM() - getLeftTargetRPM()) < Settings.Shooter.TARGET_RPM_THRESHOLD 
            && Math.abs(getRightShooterRPM() - getRightTargetRPM()) < Settings.Shooter.TARGET_RPM_THRESHOLD;
    }

    public void stop() {
        stopFeeder();
        leftTargetRPM.set(0);
        rightTargetRPM.set(0);
    }

    public abstract double getLeftShooterRPM();
    public abstract double getRightShooterRPM();

    public abstract void runFeederForwards();
    public abstract void runFeederBackwards();
    public abstract void stopFeeder();
    
    public abstract boolean hasNote();

    public abstract boolean noteShot();
}
