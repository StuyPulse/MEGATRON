package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

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
    
    private final SmartNumber targetDegrees;

    private final SmartBoolean armLimp;

    protected Arm() {
        targetDegrees = new SmartNumber("Arm/Target Angle (deg)", -90);
        armLimp = new SmartBoolean("Arm/Is Limp?", false);
    }

    // target degrees
    public void setTargetDegrees(double degrees) {
        targetDegrees.set(degrees);
    }

    public void changeTargetDegrees(double deltaDegrees) {
        setTargetDegrees(getTargetDegrees() + deltaDegrees);
    }

    public double getTargetDegrees() {
        return targetDegrees.doubleValue();
    }

    // check if at target
    public boolean atTargetDegrees(double epsilon) {
        return Math.abs(getTargetDegrees() - getDegrees()) < epsilon;
    }

    // voltage control
    public abstract void setVoltage(double voltage);

    protected abstract void setVoltageImpl(double voltage);

    // limp (voltage = 0)
    public boolean isLimp() {
        return armLimp.get();
    }

    public final void setLimp(boolean limp) {
        this.armLimp.set(limp);
    }
    
    public final void enableLimp() {
        setLimp(true);
    }

    public final void disableLimp() {
        setLimp(false);
    }

    public abstract double getDegrees();

    public abstract void stop();

    public abstract void setConstraints(double maxVelocity, double maxAcceleration);

    @Override
    public void periodic() {
    }
}