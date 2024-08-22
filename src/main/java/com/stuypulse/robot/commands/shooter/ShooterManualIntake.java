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
        shooter.setFeederState(Shooter.FeederState.INTAKING);
    }

    @Override
    public boolean isFinished() {
        return shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFeederState(Shooter.FeederState.STOP);
    }
}
