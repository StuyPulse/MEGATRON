package com.stuypulse.robot.commands.auton.BF_Series;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.shooter.SwerveDriveToShoot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FivePieceBFGH extends SequentialCommandGroup {
    
    public FivePieceBFGH(PathPlannerPath... paths) {

        addCommands(
            new ShooterScoreSpeaker(),
            
            new ShooterWaitForTarget()
                .withTimeout(1.0),

            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot(),

            new FollowPathAndIntake(paths[1]),
            new FollowPathAlignAndShoot(paths[2], new SwerveDriveToShoot()),

            new FollowPathAndIntake(paths[3]),
            new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()),

            new FollowPathAndIntake(paths[5])
        );

    }

}