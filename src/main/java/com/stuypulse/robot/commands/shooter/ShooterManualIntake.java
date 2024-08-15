package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterManualIntake extends Command{
    
    private final Shooter shooter;

    public ShooterManualIntake() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.feederIntake();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feederStop();
    }
}
