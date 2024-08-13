package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.ShooterLowFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.math.util.Units;

public class ShooterToLowFerryManual extends ShooterSetRPM {
    
    public ShooterToLowFerryManual() {
        super(new ShooterSpeeds(ShooterLowFerryInterpolation.getRPM(getFerryDistance())));
    }

    private static double getFerryDistance() {
        return Units.metersToInches(Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose()));
    }

}
