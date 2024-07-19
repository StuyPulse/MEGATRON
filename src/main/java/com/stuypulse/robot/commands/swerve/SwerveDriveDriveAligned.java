package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Assist;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.stuylib.util.AngleVelocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SwerveDriveDriveAligned extends Command {

    private final SwerveDrive swerve;
    protected final Gamepad driver;
    private final VStream velocity;

    private final SwerveRequest.FieldCentric drive;

    private final AngleController controller;
    private final IStream angleVelocity;

    public SwerveDriveDriveAligned(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
        this.driver = driver;

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

    protected double getAngleError() {
        return controller.getError().getRotation2d().getDegrees();
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
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driver.getRightX()) > Settings.Driver.Turn.DEADBAND.getAsDouble();
    }
}
