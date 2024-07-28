package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmSetShootHeightToHigh extends InstantCommand{
    
    private final Arm arm;

    public ArmSetShootHeightToHigh() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShootHeightHigh();
    }
}
