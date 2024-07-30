package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterFerryManual extends SequentialCommandGroup {
    
    public ShooterFerryManual() {
        addCommands(
            new ShooterSetRPM(Settings.Shooter.MANUAL_FERRY),
            new ShooterWaitForTarget().withTimeout((Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)),
            new ShooterFeederShoot()
        );
    }

}
