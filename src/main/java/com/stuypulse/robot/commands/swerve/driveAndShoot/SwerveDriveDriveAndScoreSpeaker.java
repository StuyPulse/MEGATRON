package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDriveDriveAndScoreSpeaker extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndScoreSpeaker(Gamepad driver) {
        super(driver, Arm.State.SPEAKER);
    }
    
    @Override
    protected Rotation2d getTargetAngle() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        return Arm.getInstance().getShootHeight() == Arm.ShootHeight.LOW
                ? speakerPose.minus(currentPose).getAngle()
                : speakerPose.minus(currentPose).getAngle().plus(Rotation2d.fromDegrees(180));
    }

    @Override
    protected double getDistanceToTarget() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        return currentPose.getDistance(speakerPose);
    }
}
