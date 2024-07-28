package com.stuypulse.robot.commands.swerve.driveAndScore;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.commands.shooter.ShooterAutoShoot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class SwerveDriveDriveAndScore extends Command {

    private final SwerveDrive swerve;
    private final Arm arm;
    private final Shooter shooter;

    private final Arm.State armState;

    protected final Gamepad driver;
    private final VStream velocity;
    private final IStream angleVelocity;

    private final BStream isAligned;

    private final SwerveRequest.FieldCentric drive;

    private final AngleController controller;

    public SwerveDriveDriveAndScore(Gamepad driver, Arm.State armState) {
        swerve = SwerveDrive.getInstance();
        shooter = Shooter.getInstance();
        arm = Arm.getInstance();
        this.driver = driver;
        this.armState = armState;

        velocity = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));
        
        drive = new SwerveRequest.FieldCentric()
            .withDeadband(Settings.Swerve.MAX_LINEAR_VELOCITY * Settings.Driver.Drive.DEADBAND.get())
            .withRotationalDeadband(Settings.Swerve.MAX_ANGULAR_VELOCITY * Settings.Driver.Turn.DEADBAND.get())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

        controller = new AnglePIDController(Settings.Swerve.Assist.kP, Settings.Swerve.Assist.kI, Settings.Swerve.Assist.kP)
            .setOutputFilter(x -> -x);
        
        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        AngleVelocity derivative = new AngleVelocity();

        angleVelocity = IStream.create(() -> derivative.get(Angle.fromRotation2d(getTargetAngle())))
            .filtered(new LowPassFilter(Assist.ANGLE_DERIV_RC))
            // make angleVelocity contribute less once distance is less than REDUCED_FF_DIST
            // so that angular velocity doesn't oscillate
            .filtered(x -> x * Math.min(1, getDistanceToTarget() / Assist.REDUCED_FF_DIST))
            .filtered(x -> -x);
        
        addRequirements(swerve);
    }

    protected abstract Rotation2d getTargetAngle();

    protected abstract double getDistanceToTarget();

    private boolean isAligned() {
        return controller.isDoneDegrees(Alignment.ANGLE_TOLERANCE.get());
    }

    @Override
    public void initialize() {
        arm.setState(armState);
        if (armState == Arm.State.SPEAKER_HIGH || armState == Arm.State.SPEAKER_LOW) {
            shooter.setTargetSpeeds(Settings.Shooter.SPEAKER);
        }
        else if (armState == Arm.State.LOW_FERRY || armState == Arm.State.LOB_FERRY) {
            shooter.setTargetSpeeds(shooter.getFerrySpeeds());
        }
        else {
            shooter.setTargetSpeeds(Settings.Shooter.SPEAKER);
        }
    }

    @Override
    public void execute() {
        swerve.setControl(
            drive.withVelocityX(velocity.get().y)
                .withVelocityY(-velocity.get().x)
                .withRotationalRate(
                    angleVelocity.get() 
                    + controller.update(
                        Angle.fromRotation2d(getTargetAngle()), 
                        Angle.fromRotation2d(swerve.getPose().getRotation()))
                )         
            );
        
        if (isAligned.get()) {
            CommandScheduler.getInstance().schedule(new ShooterAutoShoot());
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetSpeeds(new ShooterSpeeds());
        shooter.feederStop();
    }
}
