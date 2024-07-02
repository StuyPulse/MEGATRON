package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Feeder;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.constants.Settings.Shooter.PID;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.robot.constants.Settings.Shooter.Feedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax feederMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder feederEncoder;
    private final DigitalInput feederBeam;

    private final Controller leftController;
    private final Controller rightController;
    private final Controller feederController;

    private final BStream hasNote;

    protected ShooterImpl() {
        leftMotor = new CANSparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Ports.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

        leftEncoder = new FilteredRelativeEncoder(leftMotor);
        rightEncoder = new FilteredRelativeEncoder(rightMotor);
        feederEncoder = new FilteredRelativeEncoder(feederMotor);

        feederBeam = new DigitalInput(Ports.Shooter.RECIEVER_IR);
        
        leftEncoder.setVelocityConversionFactor(1.0);
        rightEncoder.setVelocityConversionFactor(1.0);
        feederEncoder.setVelocityConversionFactor(Settings.Feeder.GEARING);
        
        leftController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD)
                .setIntegratorFilter(1000, 0.5 / PID.kI));
        rightController = new MotorFeedforward(Feedforward.kS, Feedforward.kV, Feedforward.kA).velocity()
            .add(new PIDController(PID.kP, PID.kI, PID.kD)
                .setIntegratorFilter(1000, 0.5 / PID.kI));
        feederController = new MotorFeedforward(Feeder.Feedforward.kS, Feeder.Feedforward.kV, Feeder.Feedforward.kA).velocity()
            .add(new PIDController(Feeder.PID.kP, Feeder.PID.kI, Feeder.PID.kD)
                .setIntegratorFilter(1000, 1.0 / PID.kI));
        
        hasNote = BStream.create(feederBeam).filtered(new BDebounce.Both(Settings.Shooter.HAS_NOTE_DEBOUNCE));

        feederEncoder.setPositionConversionFactor(Feeder.POSITION_CONVERSION);
        feederEncoder.setPositionConversionFactor(Feeder.VELOCITY_CONVERSION);

        Motors.disableStatusFrames(leftMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(rightMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(feederMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
        //Motors.Shooter.SHOOTER_FEEDER_MOTOR.configure(feederMotor); 
        //needs to be added by Conveyor 
        
    }

    @Override
    public double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }
    
    @Override
    public double getFeederRPM() {
        return feederEncoder.getVelocity();
    }
    
    @Override
    public boolean noteInFeeder() {
        return feederBeam.get();
    }

    @Override
    public boolean noteShot() {
        return getLeftTargetRPM() > 0 &&
               getRightTargetRPM() > 0 &&
               hasNote.get() == false; 
    }

    @Override
    public void periodic () {
        super.periodic();

        leftController.update(getLeftTargetRPM(), getLeftShooterRPM());
        rightController.update(getRightTargetRPM(), getRightShooterRPM());
        feederController.update(getFeederTargetRPM(), getFeederTargetRPM());

        if (getLeftTargetRPM() == 0 && getRightTargetRPM() == 0 && getFeederTargetRPM() == 0) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
            feederMotor.stopMotor();
            
            SmartDashboard.putNumber("Shooter/Left Requested Voltage", 0);
            SmartDashboard.putNumber("Shooter/Right Requested Voltage", 0);
            SmartDashboard.putNumber("Shooter/Feeder Requested Voltage", 0);
        } else {
            leftMotor.setVoltage(leftController.getOutput());
            rightMotor.setVoltage(rightController.getOutput());
            feederMotor.setVoltage(feederController.getOutput());

            SmartDashboard.putNumber("Shooter/Left Requested Voltage", leftController.getOutput());
            SmartDashboard.putNumber("Shooter/Right Requested Voltage", rightController.getOutput());
            SmartDashboard.putNumber("Shooter/Feeder Requested Voltage", feederController.getOutput());
        }

        SmartDashboard.putNumber("Shooter/Left RPM",getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Right RPM",getRightShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM",getFeederRPM());

        SmartDashboard.putNumber("Shooter/Feeder Linear Velocity", getFeederRPM() + Units.inchesToMeters(1.0) * Math.PI);

        SmartDashboard.putNumber("Shooter/Left Error", leftController.getError());
        SmartDashboard.putNumber("Shooter/Right Error", rightController.getError());
        SmartDashboard.putNumber("Shooter/Feeder Error", feederController.getError());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederMotor.getBusVoltage() * feederMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Feeder Current", feederMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Shooter/Note Shot", noteShot());

        // SmartDashboard.putNumber("Shooter/Angle", )
        // SmartDashboard.putNumber("Shooter/Distance", ) need odometry
        
    }

}