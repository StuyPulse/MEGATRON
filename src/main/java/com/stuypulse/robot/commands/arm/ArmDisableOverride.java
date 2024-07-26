package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmDisableOverride extends InstantCommand{
    
    private final Arm arm;

    public ArmDisableOverride() {
        this.arm = Arm.getInstance();
    }

    @Override
    public void initialize() {
        arm.setOverriding(false);
    }
}
