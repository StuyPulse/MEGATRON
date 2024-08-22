package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterFeederAcquire extends InstantCommand {

    private final Shooter shooter;

    public ShooterFeederAcquire() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFeederState(Shooter.FeederState.INTAKING);
    }

}