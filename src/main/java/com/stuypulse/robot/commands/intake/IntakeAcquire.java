package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAcquire extends Command {

    private final Intake intake;

    public IntakeAcquire() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(Intake.State.ACQUIRING);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(Intake.State.STOP);
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}
