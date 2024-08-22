package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterFeederDeacquire extends InstantCommand {

    private final Shooter shooter;

    public ShooterFeederDeacquire() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setFeederState(Shooter.FeederState.DEACQUIRING);
    }
}