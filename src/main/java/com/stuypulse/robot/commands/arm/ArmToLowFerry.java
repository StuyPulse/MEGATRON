package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToLowFerry extends ArmSetState{

    public ArmToLowFerry(){
        super(Arm.State.LOW_FERRY);
    }
}