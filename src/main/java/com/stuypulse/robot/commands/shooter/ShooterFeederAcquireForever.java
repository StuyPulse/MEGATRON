package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterFeederAcquireForever extends InstantCommand {

    private final Shooter shooter;

    public ShooterFeederAcquireForever() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.runFeederForwards();
    }

}