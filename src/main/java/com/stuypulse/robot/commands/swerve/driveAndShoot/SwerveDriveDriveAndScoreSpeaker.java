package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveDriveDriveAndScoreSpeaker extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndScoreSpeaker(Gamepad driver) {
        super(driver, Arm.State.SPEAKER);
    }
    
    @Override
    protected Rotation2d getTargetAngle() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        return speakerPose.minus(currentPose).getAngle();
    }

    @Override
    protected double getDistanceToTarget() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        return currentPose.getDistance(speakerPose);
    }

    @Override
    protected ShooterSpeeds getTargetSpeeds() {
        return Settings.Shooter.SPEAKER;
    }

    @Override
    public void execute() {
        super.execute();
        if (canShoot()) {
            shooter.feederShoot();
        }
    }
}
