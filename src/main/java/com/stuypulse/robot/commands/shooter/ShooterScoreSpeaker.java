package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterScoreSpeaker extends SequentialCommandGroup {

    public ShooterScoreSpeaker() {
        addCommands(
            new ShooterSetRPM(Settings.Shooter.SPEAKER),
            new ShooterWaitForTarget()
                .andThen(new ShooterFeederAcquireForever())
        );
    }
}
