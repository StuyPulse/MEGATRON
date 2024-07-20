package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToSpeakerLow extends ArmSetState{

    public ArmToSpeakerLow(){
        super(Arm.State.SPEAKER_LOW);
    }
}