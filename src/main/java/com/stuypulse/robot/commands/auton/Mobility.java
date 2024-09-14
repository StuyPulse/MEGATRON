package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Mobility extends SequentialCommandGroup {
    
    public Mobility(PathPlannerPath... paths) {
        addCommands(
            SwerveDrive.getInstance().followPathCommand(paths[0])
        );
    }
}
