package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterFerry extends SequentialCommandGroup {
    
    public ShooterFerry() {
        addCommands(
            new ShooterSetRPM(Shooter.getInstance()::getFerrySpeeds),
            new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET),
            new ShooterFeederShoot()
        );
    }

}
