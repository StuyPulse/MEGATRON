package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToFerry extends ArmToAngle{
    public ArmToFerry(){
        super(Settings.Arm.FERRY_ANGLE.doubleValue());
    }
}