package com.stuypulse.robot.commands.auton.BCA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntake;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourPieceBCA extends SequentialCommandGroup {
    
    public FourPieceBCA(PathPlannerPath... path) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmSetState(Arm.State.FEED),

            // Drive to B + Shoot B
            new FollowPathAndIntake(path[0]),
            new ShooterAcquireFromIntake(),
            ShootRoutine.fromAnywhere().withTimeout(2.5),

            // Drive to C + Shoot C
            new FollowPathAndIntake(path[1]),
            new ShooterAcquireFromIntake(),
            ShootRoutine.fromAnywhere().withTimeout(2.5),

            // Drive to A + Shoot A
            new FollowPathAndIntake(path[2]),
            new ShooterAcquireFromIntake(),
            ShootRoutine.fromAnywhere().withTimeout(2.5)
        );
    }

}
