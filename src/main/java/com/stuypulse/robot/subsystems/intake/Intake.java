package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        FEEDING,
        SHOOTING,
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

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake/State", state.name());
    }
}