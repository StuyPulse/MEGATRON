package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    private static final Arm instance;

    static {
        instance = new ArmImpl();
    }

    public static Arm getInstance(){
        return instance;
    }

    public enum State {
        AMP,
        SUBWOOFER_SHOT,
        SPEAKER,
        FERRY,
        FEED,
        STOW,
        PRE_CLIMB,
        CLIMBING,
        RESETTING
    }

    public enum ShootHeight {
        HIGH,
        LOW
    }

    protected State state;
    protected ShootHeight shootHeight;

    protected Arm() {
        state = State.RESETTING;
        shootHeight = ShootHeight.LOW;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public void setShootHeightHigh() {
        this.shootHeight = ShootHeight.HIGH;
    }

    public void setShootHeightLow() {
        this.shootHeight = ShootHeight.LOW;
    }

    public ShootHeight getShootHeight() {
        return this.shootHeight;
    }

    public abstract boolean atTarget();

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm/State", state.toString());
    }
}