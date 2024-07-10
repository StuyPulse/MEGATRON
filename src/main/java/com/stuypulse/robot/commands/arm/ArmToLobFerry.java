package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;

public class ArmToLobFerry extends ArmToAngle{
    public ArmToLobFerry(){
        super(Settings.Arm.LOB_FERRY_ANGLE.getAsDouble());
    }
}