package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAcquireFromIntake extends Command {

    private final Shooter shooter;
    private final Intake intake;

    private final StopWatch stopWatch;
    private boolean isFeeding;

    public ShooterAcquireFromIntake() {
        shooter = Shooter.getInstance();
        intake = Intake.getInstance();

        stopWatch = new StopWatch();
        isFeeding = true;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
        shooter.feederIntake();
    }

    @Override
    public void execute() {
        if (isFeeding) {
            if (stopWatch.getTime() > Settings.Intake.HANDOFF_TIMEOUT) {
                intake.deacquire();
                shooter.feederDeacquire();
                isFeeding = false;
                stopWatch.reset();
            }
        }
        else {
            if (stopWatch.getTime() > Settings.Intake.MINIMUM_DEACQUIRE_TIME_WHEN_STUCK && intake.hasNote()) {
                intake.acquire();
                shooter.feederIntake();
                isFeeding = true;
                stopWatch.reset();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feederStop();
        intake.stop();
    }

}