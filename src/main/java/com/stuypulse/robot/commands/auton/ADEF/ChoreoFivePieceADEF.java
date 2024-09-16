package com.stuypulse.robot.commands.auton.ADEF;

import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowTrajectoryThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ChoreoFivePieceADEF extends SequentialCommandGroup{
    
    public ChoreoFivePieceADEF(ChoreoTrajectory... trajectories) {
        
        addCommands(
            // Preload Shot
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[0]),
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to D + Shoot D
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[1]),
            new FollowTrajectoryThenShoot(trajectories[2], false),
            new ArmToFeed(),

            // Drive to E + Shoot E
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[3]),
            new FollowTrajectoryThenShoot(trajectories[4], false),
            new ArmToFeed(),

            // Drive to F + Shoot F
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[5]),
            new FollowTrajectoryThenShoot(trajectories[6], true),
            new ArmToFeed()
        );
    }

}