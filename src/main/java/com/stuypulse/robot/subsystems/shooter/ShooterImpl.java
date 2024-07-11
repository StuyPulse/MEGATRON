package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax feederMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final DigitalInput feederBeam;

    private final SparkPIDController leftController;
    private final SparkPIDController rightController;

    private final BStream hasNote;

    protected ShooterImpl() {
        leftMotor = new CANSparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Ports.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

        leftEncoder = new FilteredRelativeEncoder(leftMotor);
        rightEncoder = new FilteredRelativeEncoder(rightMotor);

        feederBeam = new DigitalInput(Ports.Shooter.RECIEVER_IR);
        
        leftEncoder.setVelocityConversionFactor(1.2);
        rightEncoder.setVelocityConversionFactor(1.0);
        
        leftController = leftMotor.getPIDController();
        rightController = rightMotor.getPIDController();

        leftController.setP(Settings.Shooter.LEFT.PID.kP);
        leftController.setI(Settings.Shooter.LEFT.PID.kI);
        leftController.setD(Settings.Shooter.LEFT.PID.kD);
        leftController.setFF(Settings.Shooter.LEFT.FF.kV);

        rightController.setP(Settings.Shooter.RIGHT.PID.kP);
        rightController.setI(Settings.Shooter.RIGHT.PID.kI);
        rightController.setD(Settings.Shooter.RIGHT.PID.kD);
        rightController.setFF(Settings.Shooter.RIGHT.FF.kV);

        // leftController = new MotorFeedforward(Settings.Shooter.LEFT.FF.kS, Settings.Shooter.LEFT.FF.kV, Settings.Shooter.LEFT.FF.kA).velocity()
        //     .add(new PIDController(Settings.Shooter.LEFT.PID.kP, Settings.Shooter.LEFT.PID.kI, Settings.Shooter.LEFT.PID.kD)
        //         .setIntegratorFilter(1000, 0.5 / Settings.Shooter.LEFT.PID.kI));
        // rightController = new MotorFeedforward(Settings.Shooter.RIGHT.FF.kS, Settings.Shooter.RIGHT.FF.kV, Settings.Shooter.RIGHT.FF.kA).velocity()
        //     .add(new PIDController(Settings.Shooter.RIGHT.PID.kP, Settings.Shooter.RIGHT.PID.kI, Settings.Shooter.RIGHT.PID.kD)
        //         .setIntegratorFilter(1000, 0.5 / Settings.Shooter.RIGHT.PID.kI));
        
        hasNote = BStream.create(feederBeam).not().filtered(new BDebounce.Both(Settings.Shooter.HAS_NOTE_DEBOUNCE));

        Motors.disableStatusFrames(leftMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(rightMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(feederMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
        Motors.Shooter.FEEDER_MOTOR.configure(feederMotor);   
    }

    private double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    private double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }

    @Override
    public boolean atTargetSpeeds() {
        return Math.abs(getLeftShooterRPM() - getLeftTargetRPM()) < Settings.Shooter.TARGET_RPM_THRESHOLD 
            && Math.abs(getRightShooterRPM() - getRightTargetRPM()) < Settings.Shooter.TARGET_RPM_THRESHOLD;
    }

    private void setLeftShooterRPM(double rpm) {
        leftController.setReference(rpm, ControlType.kVelocity);
    }
    
    private void setRightShooterRPM(double rpm) {
        rightController.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public void runFeederForwards() {
        feederMotor.set(+Settings.Shooter.FEEDER_SPEED);
    }

    @Override
    public void runFeederBackwards() {
        feederMotor.set(-Settings.Shooter.FEEDER_SPEED);
    }

    @Override
    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    @Override
    public boolean hasNote() {
        return hasNote.get();
    }

    @Override
    public boolean noteShot() {
        return getLeftTargetRPM() > 0 && getRightTargetRPM() > 0 && hasNote.get() == false; 
    }

    @Override
    public void periodic () {
        super.periodic();

        setLeftShooterRPM(getLeftTargetRPM());
        setRightShooterRPM(getRightTargetRPM());

        if (getLeftTargetRPM() == 0) {
            leftMotor.stopMotor();
        }

        if (getRightTargetRPM() == 0) {
            rightMotor.stopMotor();
        }

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage());

        SmartDashboard.putBoolean("Shooter/Has Note", hasNote());

        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederMotor.getBusVoltage() * feederMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Feeder Current", feederMotor.getOutputCurrent());

        SmartDashboard.putBoolean("Shooter/Note Shot", noteShot());
    }

}