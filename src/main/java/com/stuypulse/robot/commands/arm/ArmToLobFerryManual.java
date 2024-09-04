package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToLobFerryManual extends ArmSetState{

    public ArmToLobFerryManual(){
        super(Arm.State.LOB_FERRY_MANUAL);
    }
}