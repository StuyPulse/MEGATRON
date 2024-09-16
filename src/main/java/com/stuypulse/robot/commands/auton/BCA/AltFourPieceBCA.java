package com.stuypulse.robot.commands.auton.BCA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AltFourPieceBCA extends SequentialCommandGroup {
    
    public AltFourPieceBCA(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            // Drive to B + Shoot B
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new WaitCommand(0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(1),
            new ShooterFeederStop(),
            new ArmToFeed(),

            // Drive to C + Drive to Shoot + Shoot C
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new WaitCommand(0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            ShootRoutine.fromAnywhere().withTimeout(1),
            new ShooterFeederStop(),
            new ArmToFeed(),

            // Drive to A + Drive to Shoot + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            new WaitCommand(0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathCommand(paths[4]),
            ShootRoutine.fromAnywhere().withTimeout(1),
            new ShooterFeederStop(),
            new ArmToFeed()
        );
    }

}
