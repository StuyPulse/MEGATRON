package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum State {
        DEACQUIRING,
        ACQUIRING,
        STOP
    }

    private State state;

    protected Intake() {
        this.state = State.STOP;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public abstract boolean hasNote();
}