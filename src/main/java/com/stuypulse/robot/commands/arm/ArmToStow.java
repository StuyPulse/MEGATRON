package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToStow extends ArmToAngle{
    public ArmToStow(){
        super(Settings.Arm.MIN_ANGLE.doubleValue(), Settings.Arm.MAX_ANGLE_ERROR.doubleValue());
    }
}
