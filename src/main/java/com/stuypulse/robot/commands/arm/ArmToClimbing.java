package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToClimbing extends ArmSetState{

    public ArmToClimbing(){
        super(Arm.State.CLIMBING);
    }
}