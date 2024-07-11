package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToAmp extends ArmSetState{

    public ArmToAmp(){
        super(Arm.State.AMP);
    }
}