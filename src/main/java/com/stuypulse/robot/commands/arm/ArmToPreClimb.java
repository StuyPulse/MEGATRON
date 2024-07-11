package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToPreClimb extends ArmSetState{

    public ArmToPreClimb(){
        super(Arm.State.PRE_CLIMB);
    }
}