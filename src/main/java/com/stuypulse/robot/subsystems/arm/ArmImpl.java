package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.DigitalInput;

public class ArmImpl extends Arm {
    
    private final CANSparkMax leftShoulder;
    private final CANSparkMax rightShoulder;
    private final RelativeEncoder armEncoder;
    
    private final DigitalInput bumpSwitch;
    private final BStream bumpSwitchOn;

    private final SmartNumber shoulderMaxVelocity;
    private final SmartNumber shoulderMaxAcceleration;

    protected ArmImpl() {
        leftShoulder = new CANSparkMax(Ports.Arm.LEFT_SHOULDER, MotorType.kBrushless);
        rightShoulder = new CANSparkMax(Ports.Arm.RIGHT_SHOULDER, MotorType.kBrushless);
        armEncoder = new FilteredRelativeEncoder(leftShoulder);

        bumpSwitch = new DigitalInput(Ports.Arm.BUMP_SWITCH);
        bumpSwitchOn = BStream.create(bumpSwitch).filtered(new BDebounce.Rising(Settings.Arm.BUMP_SWITCH_DEBOUNCE_TIME));

        armEncoder.setPositionConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);
        armEncoder.setVelocityConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);

        shoulderMaxVelocity = new SmartNumber("Arm/Max Velocity", Settings.Arm.TELEOP_MAX_VELOCITY.doubleValue());
        shoulderMaxAcceleration = new SmartNumber("Arm/Max Acceleration", Settings.Arm.TELEOP_MAX_ACCELERATION.doubleValue());
        
        Motors.Arm.LEFT_SHOULDER.configure(leftShoulder);
        Motors.Arm.RIGHT_SHOULDER.configure(rightShoulder);
    }

    @Override
    public void reset() {
        if (bumpSwitchOn.get()) armEncoder.setPosition(0);
    }    

    @Override
    public double getDegrees() {
        return (360 * armEncoder.getPosition());
    }

    @Override
    public void stop() {
        leftShoulder.setVoltage(0);
        rightShoulder.setVoltage(0);
        enableLimp();
    }

    // kinematic constraints
    @Override
    public void setConstraints(double maxVelocity, Number maxAcceleration) {
        shoulderMaxVelocity.set(maxVelocity);
        shoulderMaxAcceleration.set(maxAcceleration);
    }

    @Override
    protected void setVoltageImpl(double voltage) {
        leftShoulder.setVoltage(voltage);
        rightShoulder.setVoltage(voltage);
    }

}