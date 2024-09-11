package com.stuypulse.robot.commands.auton.BF_Series;

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChoreoFivePieceBFAC extends SequentialCommandGroup{

    public ChoreoFivePieceBFAC() {

        String[] paths = {};
        ArrayList<ChoreoTrajectory> choreoTrajs = new ArrayList<>();
        for (String path: paths) choreoTrajs.add(Choreo.getTrajectory(path));

        addCommands(
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(choreoTrajs.get(0)),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(choreoTrajs.get(1)),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(choreoTraj3),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(choreoTrajs.get(3)),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(choreoTraj5),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed(),

            new IntakeAcquire(),
            SwerveDrive.getInstance().choreoSwervePath(choreoTrajs.get(5)),
            new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
            SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(choreoTraj7),
            ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
            new ArmToFeed()
        );
    }
}
