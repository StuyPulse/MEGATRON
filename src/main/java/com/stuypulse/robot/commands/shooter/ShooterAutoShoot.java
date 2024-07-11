package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// automatically determines how to shoot depending on the state of the arm
public class ShooterAutoShoot extends SequentialCommandGroup {

    private final Shooter shooter;
    private final Arm.State armState;

    public ShooterAutoShoot() {
        shooter = Shooter.getInstance();
        armState = Arm.getInstance().getState();
        addRequirements(shooter);

        switch (armState) {
            case AMP:
                addCommands(new ShooterScoreAmp());
            case SPEAKER:
                addCommands(new ShooterScoreSpeaker());
            case FERRY:
                addCommands(new ShooterFerry());
            default:
                addCommands(new ShooterScoreSpeaker());
        }
    }
}
