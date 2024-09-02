package com.stuypulse.robot.commands.auton.HGF;

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
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceHGF extends SequentialCommandGroup {
    
     public FourPieceHGF(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            new ArmToSubwooferShot(),
            new ShooterSetRPM(Settings.Shooter.SPEAKER),
            new ArmWaitUntilAtTarget().alongWith(new ShooterWaitForTarget()).withTimeout(1.0),
            new ShooterFeederShoot(),

            // Drive, Intake, Shoot H
            new FollowPathAndIntake(paths[0]),
            new FollowPathAlignAndShoot(paths[1], new SwerveDriveToShoot()),

            // Drive, Intake, Shoot G
            new FollowPathAndIntake(paths[2]),
            new FollowPathAlignAndShoot(paths[3], new SwerveDriveToShoot()),

            // Drive, Intake, Shoot F
            new FollowPathAndIntake(paths[4]),
            new FollowPathAlignAndShoot(paths[5], new SwerveDriveToShoot())
        );
    }

}