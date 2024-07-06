package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
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
    // private final BStream bumpSwitchOn;

    private final SmartNumber maxVelocity;
    private final SmartNumber maxAcceleration;

    protected ArmImpl() {
        leftMotor = new CANSparkMax(Ports.Arm.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Arm.RIGHT_MOTOR, MotorType.kBrushless);
        armEncoder = new FilteredRelativeEncoder(leftMotor);

        bumpSwitch = new DigitalInput(Ports.Arm.BUMP_SWITCH);
        // bumpSwitchOn = BStream.create(bumpSwitch).filtered(new BDebounce.Rising(Settings.Arm.BUMP_SWITCH_DEBOUNCE_TIME));

        armEncoder.setPositionConversionFactor(Settings.Arm.Encoder.GEAR_RATIO); // in rotations
        armEncoder.setVelocityConversionFactor(Settings.Arm.Encoder.GEAR_RATIO);

        maxVelocity = new SmartNumber("Arm/Max Velocity", Settings.Arm.TELEOP_MAX_VELOCITY.doubleValue());
        maxAcceleration = new SmartNumber("Arm/Max Acceleration", Settings.Arm.TELEOP_MAX_ACCELERATION.doubleValue());
        
        Motors.Arm.LEFT_MOTOR.configure(leftMotor);
        Motors.Arm.RIGHT_MOTOR.configure(rightMotor);
    } 

    @Override
    public double getDegrees() {
        double angle = (360 * armEncoder.getPosition()) % 360;
        return (angle > 180 && angle < 360) ? angle - 360 : angle; // returns degrees (-180,180]
    }

    @Override
    public void stop() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
        enableLimp();
    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity.set(maxVelocity);
        this.maxAcceleration.set(maxAcceleration);
    }

    @Override
    protected void setVoltageImpl(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (bumpSwitch.get()) armEncoder.setPosition(0); // find rest position

        SmartDashboard.putNumber("Arm/Encoder Angle (deg))", getDegrees());
        SmartDashboard.putNumber("Arm/Raw Encoder Angle (rot)", armEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Left Bus Voltage (V)", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Arm/Right Bus Voltage (V)", rightMotor.getBusVoltage());

        SmartDashboard.putNumber("Arm/Left Current (amps)", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Right Current (amps)", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Arm/Left Duty Cycle", leftMotor.get());
        SmartDashboard.putNumber("Arm/Right Duty Cycle", rightMotor.get());
    }
}