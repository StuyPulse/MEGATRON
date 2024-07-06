package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.network.SmartNumber;
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
        return Math.abs(getLeftShooterRPM() - getLeftTargetRPM()) < 200.0 && Math.abs(getRightShooterRPM() - getRightTargetRPM()) < 200.0;
    }

    public abstract double getLeftShooterRPM();
    
    public abstract double getRightShooterRPM();

    public abstract void runFeeder();

    public abstract void feederOff();

    public abstract void amp();
    
    public abstract boolean noteInFeeder();

    public abstract boolean noteShot();
}
