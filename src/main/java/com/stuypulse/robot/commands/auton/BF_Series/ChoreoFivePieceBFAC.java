package com.stuypulse.robot.commands.auton.BF_Series;

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChoreoFivePieceBFAC extends SequentialCommandGroup{

    public ChoreoFivePieceBFAC(ChoreoTrajectory... trajectories) {

        addCommands(
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[0]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[1]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[2]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[3]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[4]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[5]),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().choreoSwervePath(trajectories[6]),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed()
        );
    }
}
