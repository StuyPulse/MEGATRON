package com.stuypulse.robot.commands.auton.BAC;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FourPieceBCA extends SequentialCommandGroup {
    
    public FourPieceBCA(PathPlannerPath... path) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),

            // Drive to B + Shoot B
            new FollowPathAndIntake(path[0]),
            ShootRoutine.fromAnywhere().withTimeout(2.5),

            // Drive to C + Shoot C
            new FollowPathAndIntake(path[1]),
            ShootRoutine.fromAnywhere().withTimeout(2.5),

            // Drive to A + Shoot A
            new FollowPathAndIntake(path[2]),
            ShootRoutine.fromAnywhere().withTimeout(2.5)
        );
    }

}
