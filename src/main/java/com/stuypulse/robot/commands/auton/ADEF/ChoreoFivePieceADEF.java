package com.stuypulse.robot.commands.auton.ADEF;

import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChoreoFivePieceADEF extends SequentialCommandGroup {
    
    public ChoreoFivePieceADEF(ChoreoTrajectory... trajectories) {
        
        addCommands(
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[0]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive, Intake, Shoot D
            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[1]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[2]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive, Intake, Shoot E
            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[3]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[4]), //no speaker align path command
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive, Intake, Shoot F
            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[5]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[6]), //no speaker align path command
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed()
            
        );
    }

}