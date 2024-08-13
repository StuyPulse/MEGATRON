package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveStop;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class UntilNoteShot extends SequentialCommandGroup {

    public UntilNoteShot(double timeout) {
        addCommands(
            new SwerveDriveStop(),
            new WaitUntilCommand(Shooter.getInstance()::hasNote)
                .withTimeout(timeout),
            new IntakeStop()
        );
    }

    public UntilNoteShot() {
        this(Settings.Auton.SHOOT_WAIT_DELAY.get());
    }
}