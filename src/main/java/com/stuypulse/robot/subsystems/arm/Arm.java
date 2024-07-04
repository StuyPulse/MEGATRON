package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.util.ArmEncoderFeedforward;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.feedforward.AngleArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.RobotBase;
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

    private final SmartBoolean shoulderLimp;

    private Optional<Double> shoulderVoltageOverride;

    private final Controller shoulderController;

    protected Arm() {
        targetDegrees = new SmartNumber("Arm/Target Angle (deg)", -90);
        
        shoulderLimp = new SmartBoolean("Arm/Is Shoulder Limp?", false);

        shoulderVoltageOverride = Optional.empty();

        shoulderController = new MotorFeedforward(Settings.Arm.Feedforward.kS, Settings.Arm.Feedforward.kV, Settings.Arm.Feedforward.kA).position()
        .add(new ArmEncoderFeedforward(Settings.Arm.Feedforward.kGNote))
        .add(new PIDController(Settings.Arm.PID.kP, Settings.Arm.PID.kI, Settings.Arm.PID.kD))
        .setOutputFilter(x -> {
            if (shoulderLimp()) return 0;
            else return shoulderVoltageOverride.orElse(x); 
        });
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
    public boolean atTargetDegrees(double toleranceDegrees) {
        return Math.abs(getTargetDegrees() - getDegrees()) < toleranceDegrees;
    }

    // voltage control
    public void setVoltage(double voltage) {
        shoulderVoltageOverride = Optional.of(voltage);
    }

    protected abstract void setVoltageImpl(double voltage);

    // limp (voltage = 0)
    public boolean shoulderLimp() {
        return shoulderLimp.get();
    }

    public final void setLimp(boolean shoulderLimp) {
        this.shoulderLimp.set(shoulderLimp);
    }
    
    public final void enableLimp() {
        setLimp(true);
    }

    public final void disableLimp() {
        setLimp(false);
    }

    public abstract double getDegrees();

    public abstract void stop();

    public abstract void reset();

    // public abstract boolean bumpSwitch();

    // public abstract double getVelocityRadiansPerSecond();

    // kinematic constraints
    public abstract void setConstraints(double maxVelocity, Number maxAcceleration);

    @Override
    public void periodic() {
        double shoulderTarget = getTargetDegrees();
        shoulderTarget = SLMath.clamp(shoulderTarget, Settings.Arm.MIN_ANGLE.doubleValue(), Settings.Arm.MAX_ANGLE.doubleValue());
        setTargetDegrees(shoulderTarget);
        
        shoulderController.update(getTargetDegrees(), getDegrees());
        setVoltageImpl(shoulderController.getOutput());
    }
}