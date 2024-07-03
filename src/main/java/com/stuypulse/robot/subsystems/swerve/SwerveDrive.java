/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve;

import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.modules.KrakenSwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private static final SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            new KrakenSwerveModule(FrontRight.ID, FrontRight.ABSOLUTE_OFFSET, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.DRIVE, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER, FrontRight.INVERTED),
            new KrakenSwerveModule(FrontLeft.ID, FrontLeft.ABSOLUTE_OFFSET, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.DRIVE, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER, FrontLeft.INVERTED),
            new KrakenSwerveModule(BackLeft.ID, BackLeft.ABSOLUTE_OFFSET, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.DRIVE, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER, BackLeft.INVERTED),
            new KrakenSwerveModule(BackRight.ID, BackRight.ABSOLUTE_OFFSET, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.DRIVE, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER, BackRight.INVERTED)
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final KrakenSwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final Pigeon2 gyro;
    private final FieldObject2d[] modules2D;

    protected SwerveDrive(KrakenSwerveModule... modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new Pigeon2(Ports.Gyro.ID, "*");
        modules2D = new FieldObject2d[modules.length];
    }

    private Translation2d getModuleOffsets() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModuleOffsets'");
    }
}
