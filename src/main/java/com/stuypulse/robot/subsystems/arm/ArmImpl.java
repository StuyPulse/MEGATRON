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
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
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

    private final Controller controller;
    private final MotionProfile motionProfile;

    protected ArmImpl() {
        leftMotor = new CANSparkMax(Ports.Arm.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Arm.RIGHT_MOTOR, MotorType.kBrushless);
        armEncoder = new FilteredRelativeEncoder(leftMotor);
        armEncoder.setPosition((-90 + 12.25)/360);

        bumpSwitch = new DigitalInput(Ports.Arm.BUMP_SWITCH);
        bumpSwitchTriggered = BStream.create(bumpSwitch).not().filtered(new BDebounce.Rising(Settings.Arm.BUMP_SWITCH_DEBOUNCE_TIME));

        armEncoder.setPositionConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);
        armEncoder.setVelocityConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);
        
        Motors.Arm.LEFT_MOTOR.configure(leftMotor);
        Motors.Arm.RIGHT_MOTOR.configure(rightMotor);
        
        motionProfile = new MotionProfile(
            Settings.Arm.MAX_VELOCITY, 
            Settings.Arm.MAX_ACCELERATION);

        controller = new MotorFeedforward(Settings.Arm.Feedforward.kS, Settings.Arm.Feedforward.kV, Settings.Arm.Feedforward.kA).position()
            .add(new ArmEncoderFeedforward(Settings.Arm.Feedforward.kG))
            .add(new PIDController(Settings.Arm.PID.kP, Settings.Arm.PID.kI, Settings.Arm.PID.kD))
            .setSetpointFilter(motionProfile);
    } 

    @Override
    public boolean atTarget() {
        return Math.abs(getTargetDegrees() - getDegrees()) < Settings.Arm.MAX_ANGLE_ERROR.getAsDouble();
    }

    private double getTargetDegrees() {
        switch (state) {
            case AMP:
                return Settings.Arm.AMP_ANGLE.getAsDouble();
            case SPEAKER:
                return getSpeakerAngle();
            case FERRY:
                return Settings.Arm.FERRY_ANGLE.getAsDouble(); 
            case FEED:
                return Settings.Arm.FEED_ANGLE.getAsDouble();
            case STOW:
                return Settings.Arm.MIN_ANGLE.getAsDouble();
            case PRE_CLIMB:
                return Settings.Arm.PRE_CLIMB_ANGLE.getAsDouble();
            default:
                return Settings.Arm.MIN_ANGLE.getAsDouble();   
        }
    }

    private double getSpeakerAngle() {
        return 0;
    }

    private double getDegrees() {
        return 360 * armEncoder.getPosition();
    }

    private void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();
        
        controller.update(getTargetDegrees(), getDegrees());
        setVoltage(SLMath.clamp(controller.getOutput(), -6, 6));

        if (bumpSwitchTriggered.get()) {
            armEncoder.setPosition(Settings.Arm.MIN_ANGLE.get()/360);
        }

        SmartDashboard.putNumber("Arm/Setpoint (deg)", controller.getSetpoint());
        SmartDashboard.putNumber("Arm/Error (deg)", controller.getError());
        SmartDashboard.putNumber("Arm/Output (V)", controller.getOutput());

        SmartDashboard.putBoolean("Arm/Bump Switch Triggered?", bumpSwitch.get());

        SmartDashboard.putNumber("Arm/Encoder Angle (deg))", getDegrees());
        SmartDashboard.putNumber("Arm/Raw Encoder Angle (rot)", armEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Left Bus Voltage (V)", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Arm/Right Bus Voltage (V)", rightMotor.getBusVoltage());

        SmartDashboard.putNumber("Arm/Left Current (amps)", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Right Current (amps)", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Arm/Left Duty Cycle", leftMotor.get());
        SmartDashboard.putNumber("Arm/Right Duty Cycle", rightMotor.get());

        SmartDashboard.putNumber("Arm/Arm Angle", getDegrees());
        SmartDashboard.putNumber("Arm/Shooter Angle", getDegrees() + 96); // shooter is offset 96 degrees counterclockwise from arm (thanks kevin)
    }
}