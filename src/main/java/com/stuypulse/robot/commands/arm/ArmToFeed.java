package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

/* moves the arm to the furthest possible position from stow
such that it can still receive notes from the intake*/
public class ArmToFeed extends ArmToAngle{
    public ArmToFeed(){
        super(Settings.Arm.FEED_ANGLE.doubleValue());
    }
}
