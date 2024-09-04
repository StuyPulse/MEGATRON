package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ArmEncoderFeedforward;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
        super();
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
        if (state == State.FEED) {
            return atValidFeedAngle();
        }
        return Math.abs(getTargetDegrees() - getDegrees()) < Settings.Arm.MAX_ANGLE_ERROR.get();
    }

    @Override
    public boolean atValidFeedAngle() {
        return getDegrees() > Settings.Arm.FEED_ANGLE.get() - Settings.Arm.MAX_ANGLE_ERROR.get()
                    && getDegrees() < Settings.Arm.MAX_ACCEPTABLE_FEED_ANGLE.get() + Settings.Arm.MAX_ANGLE_ERROR.get();
    }

    @Override
    public boolean atIntakeShouldShootAngle() {
        return getDegrees() < Settings.Intake.MAX_ARM_ANGLE_FOR_INTAKE_SHOOT;
    }

    private double getTargetDegrees() {
        switch (state) {
            case AMP:
                return Settings.Arm.AMP_ANGLE.get();
            case SUBWOOFER_SHOT:
                return Settings.Arm.SUBWOOFER_SHOT_ANGLE.get();
            case SPEAKER:
                return getSpeakerAngle();
            case LOW_FERRY:
                return Settings.Arm.LOW_FERRY_ANGLE.get();
            case LOW_FERRY_MANUAL:
                return Settings.Arm.LOW_FERRY_ANGLE.get();
            case LOB_FERRY:
                return Settings.Arm.LOB_FERRY_ANGLE.get();
            case LOB_FERRY_MANUAL:
                return Settings.Arm.LOB_FERRY_ANGLE.get();
            case FEED:
                return Settings.Arm.FEED_ANGLE.get();
            case STOW:
                return Settings.Arm.MIN_ANGLE.get();
            case PRE_CLIMB:
                return Settings.Arm.PRE_CLIMB_ANGLE.get();
            case CLIMBING:
                return Settings.Arm.POST_CLIMB_ANGLE.get();
            default:
                return Settings.Arm.MIN_ANGLE.get();   
        }
    }

    private double getSpeakerAngle() {
        try {
            Pose3d speakerPose = new Pose3d(
                Field.getAllianceSpeakerPose().getX(),
                Field.getAllianceSpeakerPose().getY(),
                Field.SPEAKER_MAX_HEIGHT,
                new Rotation3d()
            );

            Pose2d robotPose = SwerveDrive.getInstance().getPose();

            Pose3d armPivotPose = new Pose3d(
                robotPose.getX() + Settings.DISTANCE_FROM_TOWER_TO_CENTER_OF_ROBOT * robotPose.getRotation().getCos(),
                robotPose.getY() + Settings.DISTANCE_FROM_TOWER_TO_CENTER_OF_ROBOT * robotPose.getRotation().getSin(),
                Settings.HEIGHT_TO_ARM_PIVOT,
                new Rotation3d()
            );

            Translation3d pivotToSpeaker = speakerPose.minus(armPivotPose).getTranslation();

            double angleFromPivotToSpeaker = Units.radiansToDegrees(
                Math.atan(
                    pivotToSpeaker.getZ()
                    / pivotToSpeaker.toTranslation2d().getNorm()
                )
            );

            double angleBetweenPivotToSpeakerAndArm = Units.radiansToDegrees(Math.acos(Settings.Arm.LENGTH / pivotToSpeaker.getNorm()));

            return -(angleBetweenPivotToSpeakerAndArm - angleFromPivotToSpeaker);
        }
        catch (Exception exception) {
            exception.printStackTrace();
            return Settings.Arm.SUBWOOFER_SHOT_ANGLE.get();
        }
    }

    private double getDegrees() {
        return 360 * armEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return 360 / 60 * armEncoder.getVelocity();
    }

    private void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (bumpSwitchTriggered.get()) {
            armEncoder.setPosition(Settings.Arm.MIN_ANGLE.get()/360);
            if (state == State.RESETTING) {
                state = State.FEED;
            }
        }
        
        if (state == State.RESETTING) {
            setVoltage(-1.5);
            controller.update(Settings.Arm.MIN_ANGLE.get(), Settings.Arm.MIN_ANGLE.get());
        }
        else if (getTargetDegrees() == Settings.Arm.MIN_ANGLE.get() && bumpSwitchTriggered.get()) {
            setVoltage(0);
            controller.update(Settings.Arm.MIN_ANGLE.get(), Settings.Arm.MIN_ANGLE.get());
        }
        else {
            controller.update(SLMath.clamp(getTargetDegrees(), Settings.Arm.MIN_ANGLE.get(), Settings.Arm.MAX_ANGLE.get()), getDegrees());
            if (Shooter.getInstance().getFeederState() == Shooter.FeederState.SHOOTING && getDegrees() < Settings.Arm.MAX_ANGLE.get()) {
                setVoltage(controller.getOutput() + 0.31);
            }
            else {
                setVoltage(controller.getOutput());
            }
        }

        SmartDashboard.putNumber("Arm/Setpoint (deg)", controller.getSetpoint());
        SmartDashboard.putNumber("Arm/Error (deg)", controller.getError());
        SmartDashboard.putNumber("Arm/Output (V)", controller.getOutput());

        SmartDashboard.putBoolean("Arm/Bump Switch Triggered?", !bumpSwitch.get());

        SmartDashboard.putNumber("Arm/Encoder Angle (deg))", getDegrees());
        SmartDashboard.putNumber("Arm/Raw Encoder Angle (rot)", armEncoder.getPosition());

        SmartDashboard.putNumber("Arm/Left Bus Voltage (V)", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Arm/Right Bus Voltage (V)", rightMotor.getBusVoltage());

        SmartDashboard.putNumber("Arm/Left Current (amps)", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Right Current (amps)", rightMotor.getOutputCurrent());

        SmartDashboard.putNumber("Arm/Left Duty Cycle", leftMotor.get());
        SmartDashboard.putNumber("Arm/Right Duty Cycle", rightMotor.get());

        SmartDashboard.putNumber("Arm/Arm Angle", getDegrees());

        SmartDashboard.putBoolean("Arm/At Target", atTarget());
    }
}