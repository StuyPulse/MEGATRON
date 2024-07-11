package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToStow extends ArmSetState{

    public ArmToStow(){
        super(Arm.State.STOW);
    }
}