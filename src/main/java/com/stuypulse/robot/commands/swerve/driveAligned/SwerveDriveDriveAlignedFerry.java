package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;


public class SwerveDriveDriveAlignedFerry extends SwerveDriveDriveAligned {

    public SwerveDriveDriveAlignedFerry(Gamepad driver) {
        super(driver);
    }

    @Override
    protected Rotation2d getTargetAngle() {
        return SwerveDrive.getInstance().getPose().getTranslation().minus(Field.getAmpCornerPose()).getAngle();
    }

    @Override
    protected double getDistanceToTarget() {
        return SwerveDrive.getInstance().getPose().getTranslation().getDistance(Field.getAmpCornerPose());
    }
}
