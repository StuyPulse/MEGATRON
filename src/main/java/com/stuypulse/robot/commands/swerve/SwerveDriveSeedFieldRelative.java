package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/** reset the field-centric heading */
public class SwerveDriveSeedFieldRelative extends InstantCommand{

    private final SwerveDrive swerve;

    public SwerveDriveSeedFieldRelative() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        swerve.seedFieldRelative();
    }
}
