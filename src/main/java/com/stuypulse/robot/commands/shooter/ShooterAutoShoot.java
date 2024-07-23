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
        switch (Arm.getInstance().getActualState()) {
            case AMP:
                CommandScheduler.getInstance().schedule(new ShooterScoreAmp());
                break;
            case SPEAKER_HIGH:
                CommandScheduler.getInstance().schedule(new ShooterScoreSpeaker());
                break;
            case SPEAKER_LOW:
                CommandScheduler.getInstance().schedule(new ShooterScoreSpeaker());
                break;
            case LOW_FERRY:
                CommandScheduler.getInstance().schedule(new ShooterFerry());
                break;
            case LOB_FERRY:
                CommandScheduler.getInstance().schedule(new ShooterFerry());
                break;
            default:
                CommandScheduler.getInstance().schedule(new ShooterScoreSpeaker());
                break;
        }
    }
}
