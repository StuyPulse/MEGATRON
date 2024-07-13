package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveXMode extends Command {

    private final SwerveDrive swerve;

    public SwerveDriveXMode() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}