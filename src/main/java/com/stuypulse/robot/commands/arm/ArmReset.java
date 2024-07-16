package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmReset extends ArmSetState{
    
    public ArmReset() {
        super(Arm.State.RESETTING);
    }
}
