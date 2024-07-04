package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToAmp extends ArmToAngle{
    public ArmToAmp(){
        super(Settings.Arm.AMP_ANGLE.doubleValue());
    }
}