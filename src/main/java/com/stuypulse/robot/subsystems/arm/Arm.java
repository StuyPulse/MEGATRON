package com.stuypulse.robot.subsystems.arm;

import java.util.Optional;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ArmEncoderFeedforward;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.control.feedback.PIDController;
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

    private Optional<Double> voltageOverride;

    private final Controller controller;

    protected Arm() {
        targetDegrees = new SmartNumber("Arm/Target Angle (deg)", -90);
        
        armLimp = new SmartBoolean("Arm/Is Limp?", false);

        voltageOverride = Optional.empty();

        controller = new MotorFeedforward(Settings.Arm.Feedforward.kS, Settings.Arm.Feedforward.kV, Settings.Arm.Feedforward.kA).position()
        .add(new ArmEncoderFeedforward(Settings.Arm.Feedforward.kGNote))
        .add(new PIDController(Settings.Arm.PID.kP, Settings.Arm.PID.kI, Settings.Arm.PID.kD))
        .setOutputFilter(x -> {
            if (isLimp()) return 0;
            else return voltageOverride.orElse(x); 
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
        voltageOverride = Optional.of(voltage);
    }

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
        double target = getTargetDegrees();
        target = SLMath.clamp(target, Settings.Arm.MIN_ANGLE.doubleValue(), Settings.Arm.MAX_ANGLE.doubleValue());
        setTargetDegrees(target);
        
        controller.update(getTargetDegrees(), getDegrees());
        setVoltageImpl(controller.getOutput());

        SmartDashboard.putNumber("Arm/Setpoint (deg)", controller.getSetpoint());
        SmartDashboard.putNumber("Arm/Error (deg)", controller.getError());
        SmartDashboard.putNumber("Arm/Output (V)", controller.getOutput());
    }
}