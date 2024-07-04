package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings.Shooter;

public class FeederAcquire extends ShooterSetRPM {

    public FeederAcquire() {
        super(Shooter.FEEDER_GET);
    }
}