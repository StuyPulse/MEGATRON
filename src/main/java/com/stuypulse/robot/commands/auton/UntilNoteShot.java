package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveStop;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class UntilNoteShot extends SequentialCommandGroup {

    public UntilNoteShot(double delay) {
        addCommands(
            new InstantCommand(SwerveDrive.getInstance()::stop, SwerveDrive.getInstance()),
            new WaitUntilCommand(Shooter.getInstance()::hasNote)
                .withTimeout(delay),
            new IntakeStop()
        );
    }

    public UntilNoteShot() {
        this(Settings.Auton.SHOOT_WAIT_DELAY.get());
    }

public static Command UntilNoteShot(double timeout) {
        return new SequentialCommandGroup(
            new InstantCommand(SwerveDrive.getInstance()::stop, SwerveDrive.getInstance()),
            new WaitCommand(timeout),
            new IntakeStop()
        );
    }

}