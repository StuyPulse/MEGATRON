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
        feederStop();
        leftTargetRPM.set(0);
        rightTargetRPM.set(0);
    }

    public abstract boolean atTargetSpeeds();

    public abstract ShooterSpeeds getFerrySpeeds();

    public abstract void feederIntake();
    public abstract void feederDeacquire();
    public abstract void feederShoot();
    public abstract void feederStop();
    
    public abstract boolean hasNote();
}
