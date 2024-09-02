package com.stuypulse.robot.commands.auton.tests;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLine extends SequentialCommandGroup{
    
    public StraightLine() {
        addCommands(
            SwerveDrive.getInstance().followPathCommand("Straight Line")
        );
    }

}
