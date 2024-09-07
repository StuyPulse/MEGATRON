package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToLowFerryManual extends ArmSetState{

    public ArmToLowFerryManual(){
        super(Arm.State.LOW_FERRY_MANUAL);
    }
}