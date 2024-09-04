package com.stuypulse.robot.commands.auton.BCA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceBCA extends SequentialCommandGroup {
    
    public FourPieceBCA(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Drive to B + Shoot B
            new IntakeAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive to C + Shoot C
            new IntakeAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed()
        );
    }

}
