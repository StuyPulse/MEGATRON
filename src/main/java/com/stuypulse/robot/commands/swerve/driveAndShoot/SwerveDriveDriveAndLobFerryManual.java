package com.stuypulse.robot.commands.swerve.driveAndShoot;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveDriveDriveAndLobFerryManual extends SwerveDriveDriveAndShoot{

    public SwerveDriveDriveAndLobFerryManual(Gamepad driver) {
        super(driver, Arm.State.LOB_FERRY);
    }
    
    @Override
    protected Rotation2d getTargetAngle() {
        return Field.getManualFerryPosition().minus(Field.getAmpCornerPose()).getAngle().plus(Rotation2d.fromDegrees(180));
    }

    @Override
    protected double getDistanceToTarget() {
        return Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose());
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
