package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ReroutableFourPieceHGF extends SequentialCommandGroup {
    
    public ReroutableFourPieceHGF(PathPlannerPath... paths) {
        PathPlannerPath H_TO_G = paths[6];
        PathPlannerPath G_TO_F = paths[7];

        

    }

}
