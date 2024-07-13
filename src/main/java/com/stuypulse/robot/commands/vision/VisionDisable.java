package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionDisable extends InstantCommand {

    public VisionDisable() {}
    
    @Override
    public void initialize() {
        SwerveDrive.getInstance().setVisionEnabled(false);
    }
}
