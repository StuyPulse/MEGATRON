package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ReroutableFourPieceHGF extends SequentialCommandGroup {
    
    public ReroutableFourPieceHGF(PathPlannerPath... paths) {

        PathPlannerPath H_To_G_Reroute = paths[6];
        PathPlannerPath G_To_F_Reroute = paths[7];

        addCommands(

             // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Drive to H
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            
            new ConditionalCommand(

                // If H is successfully intaken
                new SequentialCommandGroup(
                    new FollowPathThenShoot(paths[1], false),
                    new ArmToFeed(),
    
                    // Drive to G + Shoot G
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(paths[2]),
                    new FollowPathThenShoot(paths[3], false),
                    new ArmToFeed(),
    
                    // Drive to F + Shoot F
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(paths[4]),
                    new FollowPathThenShoot(paths[5], true),
                    new ArmToFeed()),
                
                // If H is not intaken
                new SequentialCommandGroup(
                    // Redirect from H to G
                    new IntakeSetAcquire(),
                    SwerveDrive.getInstance().followPathCommand(H_To_G_Reroute),

                    // If G is successfully intaken
                    new ConditionalCommand(

                        new SequentialCommandGroup(
                            // Shoot G
                            new FollowPathThenShoot(paths[3], false),
                            new ArmToFeed(),
    
                            // Drive to F + Shoot F
                            new IntakeSetAcquire(),
                            SwerveDrive.getInstance().followPathCommand(paths[4]),
                            new FollowPathThenShoot(paths[5], true),
                            new ArmToFeed()
                        ),

                        // If G is not intaken
                        new SequentialCommandGroup(
                            // Redirect from G to F
                            new IntakeSetAcquire(),
                            SwerveDrive.getInstance().followPathCommand(G_To_F_Reroute),

                            // Shoot F
                            new FollowPathThenShoot(paths[5], true),
                            new ArmToFeed()
                        ),

                        // Runs G to F redirection if hasNote is false at G
                        Intake.getInstance()::hasNote
                    )
                ),

                // Runs H to G redirection if hasNote is false at H
                Intake.getInstance()::hasNote 
                
            )

        );
    }

}
