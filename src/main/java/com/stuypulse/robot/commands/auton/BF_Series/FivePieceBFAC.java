package com.stuypulse.robot.commands.auton.BF_Series;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FivePieceBFAC extends SequentialCommandGroup {
    
    public FivePieceBFAC(PathPlannerPath... paths) {

        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote() || Intake.getInstance().hasNote()),
            new WaitUntilCommand(() -> Shooter.getInstance().hasNote()).onlyIf(() -> Intake.getInstance().hasNote()).withTimeout(1.0),
            ShootRoutine.fromAnywhere().onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            ShootRoutine.fromAnywhere().onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            ShootRoutine.fromAnywhere().onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            ShootRoutine.fromAnywhere().onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed()
        );

    }

}
