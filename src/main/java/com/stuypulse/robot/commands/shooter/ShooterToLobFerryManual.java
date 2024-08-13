package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.math.util.Units;

public class ShooterToLobFerryManual extends ShooterSetRPM {
    
    public ShooterToLobFerryManual() {
        super(new ShooterSpeeds(ShooterLobFerryInterpolation.getRPM(getFerryDistance())));
    }

    private static double getFerryDistance() {
        return Units.metersToInches(Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose()));
    }

}
