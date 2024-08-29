package com.stuypulse.robot.commands.auton.ADEF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.DoNothingCommand;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ReroutableFivePieceADEF extends SequentialCommandGroup {
    
    public ReroutableFivePieceADEF(PathPlannerPath... paths) {

        PathPlannerPath D_DOWN = paths[7];

        addCommands(
            // Preload
            new ShooterScoreSpeaker(),
            
            new ShooterWaitForTarget()
                .withTimeout(1.0),

            // Intake + Shoot A
            new FollowPathAndIntake(paths[0]),
            new ShooterScoreSpeaker(),

            // Intake + Shoot D
            new FollowPathAndIntake(paths[1]),
            new FollowPathAlignAndShoot(paths[2], new SwerveDriveToShoot()),

            // Intake + Shoot E
            new FollowPathAndIntake(paths[3]),
            new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()),

            new ConditionalCommand(
                new SequentialCommandGroup(
                    new FollowPathAndIntake(paths[5]),
                    
                new ConditionalCommand(
                    new FollowPathAlignAndShoot(paths[6], new SwerveDriveToShoot()),
                    new DoNothingCommand(),
                    Intake.getInstance()::hasNote)),

                    new SequentialCommandGroup(
                        new FollowPathAndIntake(D_DOWN),

                        new ConditionalCommand(
                            new FollowPathAlignAndShoot(paths[7], new SwerveDriveToShoot()),
                            new DoNothingCommand(),
                            Intake.getInstance()::hasNote)
                    ),
                    Intake.getInstance()::hasNote)
                    );

    }

}
