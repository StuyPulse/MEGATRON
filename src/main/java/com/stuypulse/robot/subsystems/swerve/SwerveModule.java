package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase{
    private final String id;
    private final Translation2d offset;

    private SwerveModuleState targetState;

    public SwerveModule(String id, Translation2d offset){
        this.id = id;
        this.offset = offset;

        targetState = new SwerveModuleState();
    }
    
    private final String getID(){
        return this.id;
    }

    private final Translation2d getModuleOffset(){
        return this.offset;
    }

    public abstract double getVelocity();

    public abstract Rotation2d getAngle();

    public abstract SwerveModulePosition getModulePosition();

    public final SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    public final void setTargetState(SwerveModuleState state){
        targetState = SwerveModuleState.optimize(state, getAngle());
    }
}
