package com.stuypulse.robot.commands.auton.tests;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SquareTest extends SequentialCommandGroup {
    
    public SquareTest() {
        addCommands(
            SwerveDrive.getInstance().followPathCommand("Square Top"),
            SwerveDrive.getInstance().followPathCommand("Square Right"),
            SwerveDrive.getInstance().followPathCommand("Square Bottom"),
            SwerveDrive.getInstance().followPathCommand("Square Left")
        );
    }

}
