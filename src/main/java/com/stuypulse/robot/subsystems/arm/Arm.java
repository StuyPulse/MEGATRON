package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

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

    protected Arm() {
        targetDegrees = new SmartNumber("Arm/Target Angle", -90 + 12.25);
    }

    // target degrees
    public void setTargetDegrees(double degrees) {
        targetDegrees.set(SLMath.clamp(degrees, Settings.Arm.MIN_ANGLE.doubleValue(), Settings.Arm.MAX_ANGLE.doubleValue()));
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

    public abstract double getDegrees();

    public abstract void stop();

    public abstract void setConstraints(double maxVelocity, double maxAcceleration);

    @Override
    public void periodic() {
    }
}