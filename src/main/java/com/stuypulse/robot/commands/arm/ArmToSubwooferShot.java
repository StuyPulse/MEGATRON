package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToSubwooferShot extends ArmSetState{

    public ArmToSubwooferShot(){
        super(Arm.State.SUBWOOFER_SHOT);
    }
}