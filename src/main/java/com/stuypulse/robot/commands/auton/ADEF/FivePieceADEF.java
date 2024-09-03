package com.stuypulse.robot.commands.auton.ADEF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FivePieceADEF extends SequentialCommandGroup {
    
    public FivePieceADEF(PathPlannerPath... paths) {
        
        addCommands(
            ShootRoutine.fromSubwoofer(),

            new FollowPathAndIntake(paths[0]),
            ShootRoutine.fromAnywhere(),

            // Drive, Intake, Shoot D
            // Drive, Intake, Shoot D
            new FollowPathAndIntake(paths[1]),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[2]),
            ShootRoutine.fromAnywhere(),

            // Drive, Intake, Shoot E
            // Drive, Intake, Shoot E
            new FollowPathAndIntake(paths[3]),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[4]),
            ShootRoutine.fromAnywhere(),

            // Drive, Intake, Shoot F
            // Drive, Intake, Shoot F
            new FollowPathAndIntake(paths[5]),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[6]),
            ShootRoutine.fromAnywhere()
        );
    }

}
