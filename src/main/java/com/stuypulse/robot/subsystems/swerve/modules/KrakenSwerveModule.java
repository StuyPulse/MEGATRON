/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

public class KrakenSwerveModule extends SwerveModule {
    // TODO: Implement this later (maybe Kalimul)
    // public static SwerveModule[] getModules() {
    //     return new SwerveModule[] {
    //         // new KrakenSwerveModule("Front Right", Settings.Swerve.FrontRight.ABSOLUTE_OFFSET )
    //     }
    // }

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final CANcoder turnEncoder;  // TODO: Replace with AnalogInput
    
    // private final AngleController angleController;

    // TODO: Add status signals + odometry queues

    // TODO: Look at IFilter targetAcceleration implementation
    // TODO: Add records for nicer, more ceExecutor = Executors.newer code

    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8); // idk how many threads necessary

    // TODO: Add control from 6328
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC driveVelocityControl =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC turnPositionControl =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);

    public KrakenSwerveModule(
        String id,
        Rotation2d angleOffset,
        Translation2d positionOffset,
        int driveMotorID,
        int turnMotorID,
        int turnEncoderID,
        boolean inverted
    ) {
        
    super(id, angleOffset, positionOffset, inverted);

        this.angleOffset = angleOffset;

        driveMotor = new TalonFX(driveMotorID, "*");
        turnMotor = new TalonFX(turnMotorID, "*");
        turnEncoder = new CANcoder(turnEncoderID);


        // TODO: CONFIGURE BOTH DRIVE AND TURN MOTORS !!! from 6328 code
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Drive.MAX_FORWARD_TORQUE;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = Drive.MIN_REVERSE_TORQUE;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Drive.TORQUE_RAMP_RATE;

        driveConfig.MotorOutput.Inverted = 
            inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = Turn.MAX_FORWARD_TORQUE;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = Turn.MIN_REVERSE_TORQUE;

        turnConfig.MotorOutput.Inverted = 
            Turn.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive; 
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        driveConfig.Feedback.SensorToMechanismRatio = Drive.L4;
        turnConfig.Feedback.SensorToMechanismRatio = Turn.TURN;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        for (int i = 0; i < 4; i++) {
            boolean error = driveMotor.getConfigurator().apply(driveConfig, 0.1) == StatusCode.OK;
            error = error && (turnMotor.getConfigurator().apply(turnConfig, 0.1) == StatusCode.OK);
            if (!error) break;
        }
        
        // TODO: Add signals for motor information, apply to both drive and turn motors, optimize but utilization
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * Encoder.Drive.VELOCITY_CONVERSION;
    }

    @Override
    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble() * Encoder.Drive.POSITION_CONVERSION;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    }

    public void setDriveVolts(double voltage) {
        driveMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void setTurnVolts(double voltage) {
        turnMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void setCharacterization(double input) {
        driveMotor.setControl(currentControl.withOutput(input));
    }
    
    public void setDriveVelocity(double velocityRadsPerrSec, double feedforward) {
        driveMotor.setControl(
            driveVelocityControl
                .withVelocity(Units.radiansToRotations(velocityRadsPerrSec))
                .withFeedForward(feedforward));
    }

    public void setTurnPosition(double angleRads) {
        turnMotor.setControl(turnPositionControl.withPosition(Units.radiansToRotations(angleRads)));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModulePosition'");
    }

    public void setDrivePID(double kP, double kI, double kD) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveMotor.getConfigurator().apply(driveConfig, 0.01);
    }
    
    public void setTurnPID(double kP, double kI, double kD) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kI = kI;
        turnConfig.Slot0.kD = kD;
    }

    public void setDriveBrakeMode(boolean enable) {
        brakeModeExecutor.execute(
            () -> {
                synchronized (driveConfig) {
                    driveConfig.MotorOutput.NeutralMode = 
                        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                    driveMotor.getConfigurator().apply(driveConfig, 0.25); // maybe use timeout value
                }
        });
    }

    public void setTurnBrakeMode(boolean enable) {
        brakeModeExecutor.execute(
            () -> {
                synchronized(turnConfig) {
                    turnConfig.MotorOutput.NeutralMode = 
                        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                    turnMotor.getConfigurator().apply(turnConfig, 0.25);
                }
        });
    }

    public void stop() {
        driveMotor.setControl(neutralControl);
        turnMotor.setControl(neutralControl);
    }
    
     // TODO: Update inputs in periodic

    @Override
    public void periodic() {

    }
}

