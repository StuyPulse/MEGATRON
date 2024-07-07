package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterFerry extends SequentialCommandGroup {
    
    public ShooterFerry() {
        addCommands(
            new ShooterSetRPM(Settings.Shooter.SPEAKER),
            new ShooterWaitForTarget()
                .andThen(new ShooterFeederAcquireForever())
        );
    }

}
