package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToLowFerry extends ArmToAngle{
    public ArmToLowFerry(){
        super(Settings.Arm.LOW_FERRY_ANGLE.getAsDouble());
    }
}