package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFeed extends Command {

    private final Intake intake;

    public IntakeFeed() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(Intake.State.FEEDING);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(Intake.State.STOP);
    }

    @Override
    public boolean isFinished() {
        return Shooter.getInstance().hasNote();
    }
}
