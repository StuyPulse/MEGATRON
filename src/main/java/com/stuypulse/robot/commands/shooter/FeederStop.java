package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings.Shooter;

public class FeederStop extends ShooterSetRPM {

    public FeederStop() {
        super(Shooter.FEEDER_STOP);
    }

}