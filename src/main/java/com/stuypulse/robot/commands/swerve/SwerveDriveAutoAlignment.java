package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * automatically determines how to align depending on the state of the arm
 * 
 * <p> buzzes controller if arm is not in one of the scoring/ferrying positions
 */
public class SwerveDriveAutoAlignment extends InstantCommand{

    private final Gamepad driver;

    public SwerveDriveAutoAlignment(Gamepad driver) {
        this.driver = driver;
    }

    @Override
    public void initialize() {
        switch (Arm.getInstance().getActualState()) {
            case AMP:
                CommandScheduler.getInstance().schedule(new SwerveDriveDriveAmpAligned(driver));
                break;
            case SPEAKER_HIGH:
                CommandScheduler.getInstance().schedule(new SwerveDriveDriveAlignedSpeakerHigh(driver));
                break;
            case SPEAKER_LOW:
                CommandScheduler.getInstance().schedule(new SwerveDriveDriveAlignedSpeakerLow(driver));
                break;
            case LOW_FERRY:
                CommandScheduler.getInstance().schedule(new SwerveDriveDriveAlignedLowFerry(driver));
                break;
            case LOB_FERRY:
                CommandScheduler.getInstance().schedule(new SwerveDriveDriveAlignedLobFerry(driver));
                break;
            default:
                CommandScheduler.getInstance().schedule(new BuzzController(driver));
                break;
        }
    }
}
