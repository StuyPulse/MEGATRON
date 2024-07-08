/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {

    private final String id;

    private SwerveModuleState targetState;

    public SwerveModule(String id) {
        this.id = id;
        
        targetState = new SwerveModuleState();
    }

    public final String getId() {
        return this.id;
    }

    public abstract double getVelocity();

    public abstract Rotation2d getAngle();

    public abstract SwerveModulePosition getModulePosition();

    public final SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public final void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public final SwerveModuleState getTargetState() {
        return targetState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Velocity", getVelocity());
    }
}
