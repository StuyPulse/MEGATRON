package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterScoreAmp extends SequentialCommandGroup {

    public ShooterScoreAmp() {
        addCommands(
            new ShooterFeederDeacquire(),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasNote()),
            new WaitUntilCommand(Settings.Arm.SHOULD_RETURN_TO_FEED_TIME.get()),
            new InstantCommand(() -> Shooter.getInstance().feederStop())
        );
    }
}
