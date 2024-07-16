package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToFerry extends ArmSetState{

    public ArmToFerry(){
        super(Arm.State.FERRY);
    }
}