package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveDriveAndFerry extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndFerry(Gamepad driver) {
        super(driver, Arm.State.FERRY);
    }

    private Translation2d getAmpCornerPose() {
        Translation2d targetPose = Robot.isBlue()
            ? new Translation2d(0.0, Field.WIDTH - 1.5)
            : new Translation2d(0.0, 1.5);
        
        return targetPose;
    }
    
    @Override
    protected Rotation2d getTargetAngle() {
        return Arm.getInstance().getShootHeight() == Arm.ShootHeight.LOW
                ? SwerveDrive.getInstance().getPose().getTranslation().minus(getAmpCornerPose()).getAngle()
                : SwerveDrive.getInstance().getPose().getTranslation().minus(getAmpCornerPose()).getAngle().plus(Rotation2d.fromDegrees(180));
    }

    @Override
    protected double getDistanceToTarget() {
        return SwerveDrive.getInstance().getPose().getTranslation().getDistance(getAmpCornerPose());
    }
}
