package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveDriveDriveAndLowFerry extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndLowFerry(Gamepad driver) {
        super(driver, Arm.State.LOB_FERRY);
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

    @Override
    protected ShooterSpeeds getTargetSpeeds() {
        return shooter.getFerrySpeeds();
    }

    @Override
    public void execute() {
        super.execute();
        if (isAligned.get()) {
            CommandScheduler.getInstance().schedule(new ShooterFerry());
        }
    }
}
