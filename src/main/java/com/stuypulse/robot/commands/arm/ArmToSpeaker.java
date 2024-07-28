package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToSpeaker extends ArmSetState{

    public ArmToSpeaker(){
        super(Arm.State.SPEAKER);
    }
}