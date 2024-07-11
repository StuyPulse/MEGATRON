package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * automatically determines how to shoot depending on the state of the arm
 */
public class ShooterAutoShoot extends SequentialCommandGroup {

    public ShooterAutoShoot() {
        
        switch (Arm.getInstance().getState()) {
            case AMP:
                addCommands(
                    new ShooterScoreAmp()
                );
                break;
            case SPEAKER:
                addCommands(
                    new ShooterScoreSpeaker()
                );
                break;
            case FERRY:
                addCommands(
                    new ShooterFerry()
                );
                break;
            default:
                addCommands(
                    new ShooterScoreSpeaker()
                );
        }
    }
}
