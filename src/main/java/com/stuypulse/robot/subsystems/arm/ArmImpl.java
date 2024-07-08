package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ArmEncoderFeedforward;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder armEncoder;
    
    private final DigitalInput bumpSwitch;
    private final BStream bumpSwitchTriggered;

    private final SmartNumber maxVelocity;
    private final SmartNumber maxAcceleration;

    private Optional<Double> voltageOverride;

    private final Controller controller;

    protected ArmImpl() {
        leftMotor = new CANSparkMax(Ports.Arm.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Arm.RIGHT_MOTOR, MotorType.kBrushless);
        armEncoder = new FilteredRelativeEncoder(leftMotor);
        armEncoder.setPosition((-90 + 12.25)/360);

        bumpSwitch = new DigitalInput(Ports.Arm.BUMP_SWITCH);
        bumpSwitchTriggered = BStream.create(bumpSwitch).not().filtered(new BDebounce.Rising(Settings.Arm.BUMP_SWITCH_DEBOUNCE_TIME));

        armEncoder.setPositionConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);
        armEncoder.setVelocityConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);

        maxVelocity = new SmartNumber("Arm/Max Velocity", Settings.Arm.TELEOP_MAX_VELOCITY.doubleValue());
        maxAcceleration = new SmartNumber("Arm/Max Acceleration", Settings.Arm.TELEOP_MAX_ACCELERATION.doubleValue());
        
        Motors.Arm.LEFT_MOTOR.configure(leftMotor);
        Motors.Arm.RIGHT_MOTOR.configure(rightMotor);

        voltageOverride = Optional.empty();

        controller = new MotorFeedforward(Settings.Arm.Feedforward.kS, Settings.Arm.Feedforward.kV, Settings.Arm.Feedforward.kA).position()
            .add(new ArmEncoderFeedforward(Settings.Arm.Feedforward.kG))
            .add(new PIDController(Settings.Arm.PID.kP, Settings.Arm.PID.kI, Settings.Arm.PID.kD))
            .setOutputFilter(x -> voltageOverride.orElse(x));
    } 

    @Override
    public double getDegrees() {
        double angle = (360 * armEncoder.getPosition());
        return (angle > 180 && angle < 360) ? angle - 360 : angle; // returns degrees (-180,180]
    }

    @Override
    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity.set(maxVelocity);
        this.maxAcceleration.set(maxAcceleration);
    }

    @Override
    public void setVoltage(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    @Override
    protected void setVoltageImpl(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        double target = getTargetDegrees();
        target = SLMath.clamp(target, Settings.Arm.MIN_ANGLE.doubleValue(), Settings.Arm.MAX_ANGLE.doubleValue());
        setTargetDegrees(target);
        
        controller.update(getTargetDegrees(), getDegrees());
        setVoltageImpl(SLMath.clamp(controller.getOutput(), -6, 6));

        SmartDashboard.putNumber("Arm/Setpoint (deg)", controller.getSetpoint());
        SmartDashboard.putNumber("Arm/Error (deg)", controller.getError());
        SmartDashboard.putNumber("Arm/Output (V)", controller.getOutput());

        if (bumpSwitchTriggered.get()) {
            armEncoder.setPosition((-90 + 12.25)/360);
        }

        SmartDashboard.putBoolean("Arm/Bump Switch Triggered?", bumpSwitch.get());

        SmartDashboard.putNumber("Arm/Encoder Angle (deg))", getDegrees());
        SmartDashboard.putNumber("Arm/Raw Encoder Angle (rot)", armEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Left Bus Voltage (V)", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Arm/Right Bus Voltage (V)", rightMotor.getBusVoltage());

        SmartDashboard.putNumber("Arm/Left Current (amps)", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Right Current (amps)", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Arm/Left Duty Cycle", leftMotor.get());
        SmartDashboard.putNumber("Arm/Right Duty Cycle", rightMotor.get());

        SmartDashboard.putNumber("Arm/Target Angle", getTargetDegrees());
        SmartDashboard.putNumber("Arm/Arm Angle", getDegrees());
        SmartDashboard.putNumber("Arm/Shooter Angle", getDegrees() + 96); // shooter is offset 96 degrees counterclockwise from arm (thanks kevin)
    }
}