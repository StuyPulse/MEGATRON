package com.stuypulse.robot.commands.auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;


public class FollowTrajectoryThenShoot extends SequentialCommandGroup{

    private final double totalPathTime;
    
    public FollowTrajectoryThenShoot(ChoreoTrajectory trajectory, boolean isLastShot) {
        // totalPathTime = trajectory.getTrajectory(new ChassisSpeeds(), trajectory.getPoses()[0].getRotation()).getTotalTimeSeconds();
        totalPathTime = trajectory.getTotalTime();
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().choreoSwervePath(trajectory),
                new WaitCommand(totalPathTime > 1.5 ? totalPathTime - 1.0 : 0)
                    // wait for handoff
                    .andThen(new WaitUntilCommand(() -> Shooter.getInstance().hasNote()).onlyIf(() -> Intake.getInstance().hasNote()).alongWith(new WaitCommand(0.75)))
                    .andThen(new ArmToSpeaker().onlyIf(() -> Shooter.getInstance().hasNote()))
            ),
            isLastShot ? ShootRoutine.fromAnywhereLastShot()
                    : ShootRoutine.fromAnywhere().onlyIf(() -> Shooter.getInstance().hasNote())
        );
    }
}
