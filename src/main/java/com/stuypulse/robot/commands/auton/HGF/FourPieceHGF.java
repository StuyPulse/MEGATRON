package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceHGF extends SequentialCommandGroup {
    
     public FourPieceHGF(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Drive to H + Shoot H
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[1]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive to G + Shoot G
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[3]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive to F + Shoot F
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[4]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[5]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed()
        );
    }

}