package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignToSpeaker;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public abstract class ShootRoutine {

    public static Command fromSubwoofer() {
        return new SequentialCommandGroup(
            new ArmToSubwooferShot(),
            new ArmWaitUntilAtTarget().alongWith(new ShooterWaitForTarget()).withTimeout(1.0),
            new ShooterFeederShoot(),
            new WaitCommand(0.25) // give time for note to leave shooter
        );
    }

    public static Command fromAnywhere() {
        return new SwerveDriveAlignToSpeaker()
            .alongWith(new ArmToSpeaker()
                .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                        .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                .andThen(new WaitUntilCommand(() -> SwerveDrive.getInstance().isAlignedToSpeaker()))
                .andThen(new ShooterFeederShoot())
                .andThen(new WaitCommand(0.25)) // give time for note to leave shooter
            )
            .alongWith(new LEDSet(LEDInstructions.SPEAKER_ALIGN));
    }

}