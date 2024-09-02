package com.stuypulse.robot.commands.auton.ADEF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FivePieceADEF extends SequentialCommandGroup {
    
    public FivePieceADEF(PathPlannerPath... paths) {
        
        addCommands(
            // Preload Shot
            new ArmToSubwooferShot(),
            new ShooterSetRPM(Settings.Shooter.SPEAKER),
            new ArmWaitUntilAtTarget().alongWith(new ShooterWaitForTarget()).withTimeout(1.0),
            new ShooterFeederShoot(),

            // Drive, Intake, Shoot A
            new FollowPathAndIntake(paths[0]),
            new SwerveDriveToShoot(),
            new ShootRoutine(),

            // Drive, Intake, Shoot D
            new FollowPathAndIntake(paths[1]),
            new FollowPathAlignAndShoot(paths[2], new SwerveDriveToShoot()),

            // Drive, Intake, Shoot E
            new FollowPathAndIntake(paths[3]),
            new FollowPathAlignAndShoot(paths[4], new SwerveDriveToShoot()),

            // Drive, Intake, Shoot F
            new FollowPathAndIntake(paths[5]),
            new FollowPathAlignAndShoot(paths[6], new SwerveDriveToShoot())
        );
    }

}
