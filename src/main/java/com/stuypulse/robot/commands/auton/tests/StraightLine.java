package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.vision.VisionChangeWhiteList;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StraightLine extends SequentialCommandGroup {
    
    public StraightLine(PathPlannerPath... paths) {
        addCommands(
            new VisionChangeWhiteList(7, 8),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new VisionReloadWhiteList()
        );
    }

}
