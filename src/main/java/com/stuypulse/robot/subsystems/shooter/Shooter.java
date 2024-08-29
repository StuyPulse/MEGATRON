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

    public enum FeederState {
        INTAKING,
        DEACQUIRING,
        SHOOTING,
        STOP
    }

    private FeederState feederState;
    
    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    public Shooter() {
        feederState = FeederState.STOP;
        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", Settings.Shooter.ALWAYS_KEEP_AT_SPEED ? Settings.Shooter.SPEAKER.getLeftRPM() : 0);
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", Settings.Shooter.ALWAYS_KEEP_AT_SPEED ? Settings.Shooter.SPEAKER.getRightRPM() : 0);
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
        setFeederState(FeederState.STOP);
        leftTargetRPM.set(0);
        rightTargetRPM.set(0);
    }

    public void setFeederState(FeederState feederState) {
        this.feederState = feederState;
    }

    public FeederState getFeederState() {
        return feederState;
    }

    public abstract boolean atTargetSpeeds();

    public abstract ShooterSpeeds getFerrySpeeds();
    
    public abstract boolean hasNote();
}
