package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    // CHANGE LATER BUT FOR TESTING WEDNESDAY, INTAKEIMPL ONLY
    // static {
    //     if (Robot.isReal()) { // forgot
    //         instance = new IntakeImpl();
    //     } else {
    //         instance = new IntakeSim();
    //     }
    // }
    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    protected Intake() {}

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stopIntake();

    public abstract void funnel();

    public abstract void defunnel();

    public abstract void stopFunnel();

    public abstract double getIntakeRollerSpeed();

    public abstract double getTopFunnelRollerSpeed();

    public abstract double getLowFunnelRollerSpeed();


}