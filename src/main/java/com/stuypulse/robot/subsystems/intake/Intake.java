package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    static {
        if (Robot.isReal()) {
            instance = new IntakeImpl();
        } else {
            instance = new IntakeSim();
        }
    }

    public static Intake getInstance() {
        return instance;
    }

    protected Intake() {}

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    public abstract boolean hasNote();
}