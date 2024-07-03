/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {
    private String id;
    protected final Rotation2d angleOffset;
    private final boolean inverted;

    private SwerveModuleState targetState;

    // TODO: figure out if we need a translation offset for the swerve modules
    //       and if we do, how we will use it

    public SwerveModule(String id, Rotation2d angleOffset, boolean inverted) {
        this.id = id;
        this.angleOffset = angleOffset;
        this.inverted = inverted;

        targetState = new SwerveModuleState();
    }

    public final String getID() {
        return this.id;
    }

    public final Rotation2d getModuleOffset() {
        return this.angleOffset;
    }

    public final boolean getInverted() {
        return this.inverted;
    }

    public abstract double getVelocity();

    public abstract double getPosition();

    public abstract Rotation2d getAngle();

    // public abstract SwerveModulePosition getModulePosition();

    public final SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public final void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public final SwerveModuleState getTargetState() {
        return targetState;
    }

}