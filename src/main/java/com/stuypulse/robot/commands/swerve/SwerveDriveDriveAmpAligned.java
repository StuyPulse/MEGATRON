package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;


public class SwerveDriveDriveAmpAligned extends SwerveDriveDriveAligned {

    public SwerveDriveDriveAmpAligned(Gamepad driver) {
        super(driver);
    }


    @Override
    protected Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(Robot.isBlue() ? 90 : 270);
    }

    @Override
    protected double getDistanceToTarget() {
        return Settings.Swerve.Assist.AMP_WALL_SCORE_DISTANCE;
    }
}
