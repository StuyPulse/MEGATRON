package com.stuypulse.robot.commands.shooter;

import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetRPM extends InstantCommand {

    private final Shooter shooter;
    private final Supplier<ShooterSpeeds> speeds;

    public ShooterSetRPM(Supplier<ShooterSpeeds> speeds) {
        shooter = Shooter.getInstance();
        this.speeds = speeds;
        addRequirements(shooter);
    }

    public ShooterSetRPM(ShooterSpeeds speeds) {
        this(() -> speeds);
    }

    @Override
    public void initialize() {
        shooter.setTargetSpeeds(speeds.get());
    }

}