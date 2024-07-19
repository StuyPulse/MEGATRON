package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToSpeakerHigh extends ArmSetState{

    public ArmToSpeakerHigh(){
        super(Arm.State.SPEAKER_HIGH);
    }
}