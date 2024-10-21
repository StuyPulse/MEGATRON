package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RerouteTest extends SequentialCommandGroup {
    
    public RerouteTest(PathPlannerPath... paths) {

        PathPlannerPath B_To_A_Reroute = paths[2];
        PathPlannerPath A_To_Center = paths[3];

        addCommands(
            // Preload Shot
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new FollowPathThenShoot(paths[1], false),
                    new ArmToFeed()
                ),

                new SequentialCommandGroup(
                    SwerveDrive.getInstance().followPathCommand(B_To_A_Reroute),
                    new FollowPathThenShoot(A_To_Center, false),
                    new ArmToFeed()
                ), 

                Intake.getInstance()::hasNote

                )
        );
    }

}
