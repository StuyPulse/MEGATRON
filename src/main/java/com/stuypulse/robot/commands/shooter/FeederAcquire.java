package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FeederAcquire extends InstantCommand {

    private Shooter shooter;

    public FeederAcquire() {
        shooter = Shooter.getInstance();
            addRequirements(shooter);
    }

    public void initalize() {
        shooter.runFeeder();
    }

}