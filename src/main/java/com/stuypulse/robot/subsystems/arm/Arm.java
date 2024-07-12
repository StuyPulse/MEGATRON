package com.stuypulse.robot.subsystems.arm;

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
        SPEAKER,
        FERRY,
        FEED,
        STOW,
        PRE_CLIMB
    }

    protected State state;

    protected Arm() {
        state = State.STOW;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public abstract boolean atTarget();

    public abstract void setConstraints(double maxVelocity, double maxAcceleration);

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm/State", getState().toString());
    }
}