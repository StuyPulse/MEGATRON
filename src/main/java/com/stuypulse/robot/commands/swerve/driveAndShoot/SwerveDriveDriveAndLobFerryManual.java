package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.commands.shooter.ShooterLobFerryManual;
import com.stuypulse.robot.commands.shooter.ShooterLowFerryManual;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterLowFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveDriveDriveAndLobFerryManual extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndLobFerryManual(Gamepad driver) {
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
        return Field.getManualFerryPosition().minus(getAmpCornerPose()).getAngle().plus(Rotation2d.fromDegrees(180));
    }

    @Override
    protected double getDistanceToTarget() {
        return Field.getManualFerryPosition().getDistance(getAmpCornerPose());
    }

    @Override
    protected ShooterSpeeds getTargetSpeeds() {
        return new ShooterSpeeds(ShooterLobFerryInterpolation.getRPM(Units.metersToInches(getDistanceToTarget())));
    }

    @Override
    public void execute() {
        super.execute();
        if (canShoot()) {
            shooter.feederShoot();
        }
    }
}
