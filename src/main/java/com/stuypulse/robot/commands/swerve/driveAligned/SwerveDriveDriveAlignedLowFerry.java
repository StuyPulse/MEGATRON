package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveDriveDriveAlignedLowFerry extends SwerveDriveDriveAligned {

    public SwerveDriveDriveAlignedLowFerry(Gamepad driver) {
        super(driver);
    }

    private Translation2d getAmpCornerPose() {
        Translation2d targetPose = Robot.isBlue()
            ? new Translation2d(0.0, Field.WIDTH - 1.5)
            : new Translation2d(0.0, 1.5);
        
        return targetPose;
    }

    @Override
    protected Rotation2d getTargetAngle() {
        return SwerveDrive.getInstance().getPose().getTranslation().minus(getAmpCornerPose()).getAngle();
    }

    @Override
    protected double getDistanceToTarget() {
        return SwerveDrive.getInstance().getPose().getTranslation().getDistance(getAmpCornerPose());
    }
}
