package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmEnableOverride extends InstantCommand{
    
    private final Arm arm;

    public ArmEnableOverride() {
        this.arm = Arm.getInstance();
    }

    @Override
    public void initialize() {
        arm.setOverriding(true);
    }
}
