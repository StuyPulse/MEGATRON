package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToFeed extends ArmSetState{

    public ArmToFeed(){
        super(Arm.State.FEED);
    }
}