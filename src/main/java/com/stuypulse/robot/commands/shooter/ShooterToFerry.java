package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

public class ShooterToFerry extends ShooterSetRPM {
    
    public ShooterToFerry() {
        super(Shooter.getInstance()::getFerrySpeeds);
    }

}
