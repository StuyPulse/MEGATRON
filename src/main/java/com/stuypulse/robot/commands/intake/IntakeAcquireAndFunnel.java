package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAcquireAndFunnel extends Command {

    private final Intake intake;

    public IntakeAcquireAndFunnel() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
        intake.funnel();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        intake.stopFunnel();
    }

    @Override
    public boolean isFinished() {
        // idk 100% abt this return
        return super.isFinished();
    }
    
}
