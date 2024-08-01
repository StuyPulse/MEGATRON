package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToLobFerry extends ArmSetState{

    public ArmToLobFerry(){
        super(Arm.State.LOB_FERRY);
    }
}