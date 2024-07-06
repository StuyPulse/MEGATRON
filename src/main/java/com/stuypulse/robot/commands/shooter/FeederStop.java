package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FeederStop extends InstantCommand {

    private Shooter shooter;

    public FeederStop() {
        shooter = Shooter.getInstance();
            addRequirements(shooter);
    }
        
    public void initalize() {
        shooter.feederOff();
    }
           
}

