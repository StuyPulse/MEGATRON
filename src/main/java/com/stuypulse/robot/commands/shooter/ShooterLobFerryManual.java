package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterLobFerryManual extends SequentialCommandGroup {
    
    public ShooterLobFerryManual() {
        addCommands(
            new ShooterSetRPM(new ShooterSpeeds(ShooterLobFerryInterpolation.getRPM(getFerryDistance()))),
            new ShooterWaitForTarget().withTimeout((Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)),
            new ShooterFeederShoot()
        );
    }

    private double getFerryDistance() {
        return Units.metersToInches(Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose()));
    }

}