package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAcquireFromIntake extends Command {

    private final Shooter shooter;
    private final Intake intake;

    public ShooterAcquireFromIntake() {
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
        shooter.feederIntake();
    }

    @Override
    public boolean isFinished() {
        return shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feederStop();
        intake.stop();
    }

}