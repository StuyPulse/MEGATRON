// package com.stuypulse.robot.commands.auton.BF_Series;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.arm.ArmToFeed;
// import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
// import com.stuypulse.robot.commands.auton.ShootRoutine;
// import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
// import com.stuypulse.robot.commands.swerve.SwerveDriveToShoot;
// import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class FivePieceBFGH extends SequentialCommandGroup {
    
//     public FivePieceBFGH(PathPlannerPath... paths) {

//         addCommands(
//             ShootRoutine.fromSubwoofer(),
//             new ArmToFeed(),

//             new FollowPathAndIntake(paths[0]),
//             ShootRoutine.fromAnywhere(),

//             new FollowPathAndIntake(paths[1]),
//             SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[2]),
//             ShootRoutine.fromAnywhere(),

//             new FollowPathAndIntake(paths[3]),
//             SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[4]),
//             ShootRoutine.fromAnywhere(),

//             new FollowPathAndIntake(paths[5]),
//             SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[6]),
//             ShootRoutine.fromAnywhere()
//         );

//     }

// }
