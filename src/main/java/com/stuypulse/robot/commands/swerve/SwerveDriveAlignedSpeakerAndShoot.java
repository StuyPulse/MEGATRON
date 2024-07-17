package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

public class SwerveDriveAlignedSpeakerAndShoot extends SwerveDriveDriveAlignedSpeaker{

    private Shooter shooter;
    private Arm arm;
    private BStream canShoot;
    
    public SwerveDriveAlignedSpeakerAndShoot(Gamepad driver) {
        super(driver);
        this.shooter = Shooter.getInstance();
        this.arm = Arm.getInstance();
        this.canShoot = BStream.create(() -> getAngleError() < getAngleTolerance())
                        .and(() -> Arm.getInstance().atTarget())
                        .and(() -> Shooter.getInstance().atTargetSpeeds())
                        .filtered(new BDebounce.Both(.4));
        addRequirements(shooter, arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        arm.setState(Arm.State.SPEAKER);
        shooter.setTargetSpeeds(Settings.Shooter.SPEAKER);
    }

    private double getAngleTolerance() {
        double distance = getDistanceToTarget();
        double SHOT_LANDING_TOLERANCE = 1.0;

        return Math.toDegrees(Math.atan(SHOT_LANDING_TOLERANCE / distance));
    }

    @Override
    public void execute() {
        super.execute();
        if (canShoot.get()) {
            shooter.feederShoot();
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || arm.getState() != Arm.State.SPEAKER || !Shooter.getInstance().hasNote();
    }
}
