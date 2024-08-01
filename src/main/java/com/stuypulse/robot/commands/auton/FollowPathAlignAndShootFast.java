package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowPathAlignAndShootFast extends SequentialCommandGroup {
    
    public FollowPathAlignAndShootFast(PathPlannerPath path, FastAlignShootSpeakerRelative alignCommand) {
        addCommands(
            SwerveDrive.getInstance().followPathCommand(path),
            alignCommand,
            new IntakeStop()
        );
    }
}