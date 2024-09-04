package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeWaitUntilHasNote extends WaitUntilCommand {

    public IntakeWaitUntilHasNote() {
        super(() -> Intake.getInstance().hasNote());
    }
}
