// package com.stuypulse.robot.commands.shooter;

// import com.stuypulse.robot.subsystems.odometry.Odometry;
// import com.stuypulse.robot.subsystems.vision.NoteVision;

// import com.stuypulse.robot.subsystems.shooter.Shooter;
// import com.stuypulse.robot.subsystems.shooter.ShooterImpl;


// import com.revrobotics.CANSparkMax;
// import com.stuypulse.robot.commands.shooter.FeederAcquire;
// import edu.wpi.first.wpilibj2.command.Command;

// import com.stuypulse.robot.constants.Settings;
// import com.stuypulse.robot.constants.Settings.Feeder;
// import com.stuypulse.robot.constants.Settings.Motors;
// import com.stuypulse.robot.constants.Ports;

// // intake + shoot + intake second note + shoot. feeder + shooter should be running. only use when 2+ notes are close?

// public class RapidShooter extends Command() {

//     private final NoteVision noteVision;
//     private final ShooterImpl feederMotor;

//     public RapidShooter() {

//         if (withinIntakePath()) {
//             noteVision = NoteVision.getInstance();
            


//         }

//     }

// }