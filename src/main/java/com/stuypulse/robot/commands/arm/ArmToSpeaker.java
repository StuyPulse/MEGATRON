package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToSpeaker extends ArmToAngle{
    public ArmToSpeaker(){
        super(Settings.Arm.SPEAKER_ANGLE.doubleValue(), Settings.Arm.MAX_ANGLE_ERROR.doubleValue());
    }
}