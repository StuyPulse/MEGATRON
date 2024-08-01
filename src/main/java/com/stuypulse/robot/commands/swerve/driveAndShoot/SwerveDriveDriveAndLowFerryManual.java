package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.shooter.ShooterFerry;
import com.stuypulse.robot.commands.shooter.ShooterLowFerryManual;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ShooterLowFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SwerveDriveDriveAndLowFerryManual extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndLowFerryManual(Gamepad driver) {
        super(driver, Arm.State.LOW_FERRY);
    }
    
    @Override
    protected Rotation2d getTargetAngle() {
        return Field.getManualFerryPosition().minus(Field.getAmpCornerPose()).getAngle();
    }

    @Override
    protected double getDistanceToTarget() {
        return Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose());
    }

    @Override
    protected ShooterSpeeds getTargetSpeeds() {
        return new ShooterSpeeds(ShooterLowFerryInterpolation.getRPM(Units.metersToInches(getDistanceToTarget())));
    }

    @Override
    public void execute() {
        super.execute();
        if (canShoot()) {
            shooter.feederShoot();
        }
    }
}
