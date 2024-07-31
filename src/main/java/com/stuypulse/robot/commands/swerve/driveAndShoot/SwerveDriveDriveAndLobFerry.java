package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveDriveAndLobFerry extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndLobFerry(Gamepad driver) {
        super(driver, Arm.State.LOB_FERRY);
    }
    
    @Override
    protected Rotation2d getTargetAngle() {
        return SwerveDrive.getInstance().getPose().getTranslation().minus(Field.getAmpCornerPose()).getAngle().plus(Rotation2d.fromDegrees(180));
    }

    @Override
    protected double getDistanceToTarget() {
        return SwerveDrive.getInstance().getPose().getTranslation().getDistance(Field.getAmpCornerPose());
    }

    @Override
    protected ShooterSpeeds getTargetSpeeds() {
        return shooter.getFerrySpeeds();
    }

    @Override
    public void execute() {
        super.execute();
        if (canShoot()) {
            shooter.feederShoot();
        }
    }
}
