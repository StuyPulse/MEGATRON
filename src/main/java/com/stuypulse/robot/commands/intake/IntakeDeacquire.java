package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDeacquire extends Command {
    
    private final Intake intake;

    public IntakeDeacquire() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(Intake.State.DEACQUIRING);
    }
}
