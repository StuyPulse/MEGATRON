package com.stuypulse.robot.commands.auton.tests;

import com.choreo.lib.Choreo;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoStraightLine extends SequentialCommandGroup{
    
    public ChoreoStraightLine() {
        addCommands(
            SwerveDrive.getInstance().choreoSwervePath(Choreo.getTrajectory("Straight Line"))
        );
    }

}
