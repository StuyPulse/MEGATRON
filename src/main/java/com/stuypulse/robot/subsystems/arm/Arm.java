package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    /* SINGLETON */
    private static final Arm instance;

    static {
        instance = new ArmImpl();
    }

    public static Arm getInstance(){
        return instance;
    }

    public enum State {
        AMP,
        SPEAKER_HIGH,
        SPEAKER_LOW,
        LOW_FERRY,
        LOB_FERRY,
        FEED,
        STOW,
        PRE_CLIMB,
        RESETTING
    }

    protected State requestedState;
    protected State actualState;
    protected boolean overriding;

    protected Arm() {
        requestedState = State.RESETTING;
        actualState = State.RESETTING;
        overriding = false;
    }

    public void setRequestedState(State state) {
        this.requestedState = state;
    }

    public State getRequestedState() {
        return this.requestedState;
    }

    public State getActualState() {
        return this.actualState;
    }

    public void setOverriding(boolean overriding) {
        this.overriding = overriding;
    }

    public boolean isOverriding() {
        return this.overriding;
    }

    public abstract boolean atTarget();

    public abstract boolean shouldReturnToFeed();

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm/Is Overriding", overriding);
        SmartDashboard.putString("Arm/Requested State", getRequestedState().toString());
        SmartDashboard.putString("Arm/Actual State", getActualState().toString());
    }
}