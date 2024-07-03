package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAndFunnelStop extends Command {

    private final Intake intake;
    
    public IntakeAndFunnelStop() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stopIntake();
        intake.stopFunnel();
    }

}
