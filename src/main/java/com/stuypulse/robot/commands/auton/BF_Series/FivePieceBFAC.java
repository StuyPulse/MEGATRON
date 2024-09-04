// package com.stuypulse.robot.commands.auton.BF_Series;

// import com.pathplanner.lib.path.PathPlannerPath;
// import com.stuypulse.robot.commands.arm.ArmToFeed;
// import com.stuypulse.robot.commands.auton.ShootRoutine;
// import com.stuypulse.robot.commands.intake.IntakeAcquire;
// import com.stuypulse.robot.subsystems.shooter.Shooter;
// import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// public class FivePieceBFAC extends SequentialCommandGroup {
    
//     public FivePieceBFAC(PathPlannerPath... paths) {

//         addCommands(
//             ShootRoutine.fromSubwoofer(),
//             new ArmToFeed(),

//             new IntakeAcquire(),
//             SwerveDrive.getInstance().followPathCommand(paths[0]),
//             new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
//             ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
//             new ArmToFeed(),

//             new IntakeAcquire(),
//             SwerveDrive.getInstance().followPathCommand(paths[1]),
//             new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
//             SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[2]),
//             ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
//             new ArmToFeed(),

//             new IntakeAcquire(),
//             SwerveDrive.getInstance().followPathCommand(paths[3]),
//             new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
//             SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[4]),
//             ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
//             new ArmToFeed(),

//             new IntakeAcquire(),
//             SwerveDrive.getInstance().followPathCommand(paths[5]),
//             new WaitCommand(1.0).until(() -> Shooter.getInstance().hasNote()),
//             SwerveDrive.getInstance().followPathWithSpeakerAlignCommand(paths[6]),
//             ShootRoutine.fromAnywhere().withTimeout(2.5).onlyIf(() -> Shooter.getInstance().hasNote()),
//             new ArmToFeed()
//         );

//     }

// }
