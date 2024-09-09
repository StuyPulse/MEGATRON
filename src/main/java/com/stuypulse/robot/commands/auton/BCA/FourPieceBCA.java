package com.stuypulse.robot.commands.auton.BCA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.constants.Settings.Alignment.Shoot;
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
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new WaitCommand(0.8).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            ShootRoutine.fromAnywhere().withTimeout(2.5),
            new ShooterFeederStop(),
            new ArmToFeed(),

            // Drive to C + Shoot C
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new WaitCommand(0.8).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            ShootRoutine.fromAnywhere().withTimeout(2.5),
            new ShooterFeederStop(),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[4]),
            new WaitCommand(0.8).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathCommand(paths[5]),
            ShootRoutine.fromAnywhere().withTimeout(2.5),
            new ShooterFeederStop(),
            new ArmToFeed()
        );
    }

}
