package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * automatically determines how to shoot depending on the state of the arm
 */
public class ShooterAutoShoot extends InstantCommand{

    @Override
    public void initialize() {
        switch (Arm.getInstance().getState()) {
            case AMP:
                CommandScheduler.getInstance().schedule(new ShooterScoreAmp());
            case SPEAKER:
                CommandScheduler.getInstance().schedule(new ShooterScoreSpeaker());
            case FERRY:
                CommandScheduler.getInstance().schedule(new ShooterFerry());
            default:
                CommandScheduler.getInstance().schedule(new ShooterScoreSpeaker());
        }
    }
}
