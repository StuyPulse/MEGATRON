package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeShoot extends SequentialCommandGroup {

    public IntakeShoot() {
        addCommands(
            new InstantCommand(() -> Intake.getInstance().setState(Intake.State.SHOOTING), Intake.getInstance()),
            new WaitCommand(Settings.Intake.INTAKE_SHOOT_TIME),
            new IntakeStop()
        );
    }
}
