package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.numbers.filters.RateLimit;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command {

    private final SwerveDrive swerve;

    private final Gamepad driver;

    private final SwerveRequest.FieldCentric drive;

    private final VStream speed;
    private final IStream turn;

    public SwerveDriveDrive(Gamepad driver) {
        swerve = SwerveDrive.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.clamp(x, -1, 1),
                x -> SLMath.spow(x, Turn.POWER.get()),
                x -> x * Turn.MAX_TELEOP_TURN_SPEED.get(),
                new RateLimit(Turn.MAX_TELEOP_TURN_ACCEL.get()),
                new LowPassFilter(Turn.RC.get()));

        this.driver = driver;

        drive = new SwerveRequest.FieldCentric()
            .withDeadband(Settings.Swerve.MAX_LINEAR_VELOCITY * Settings.Driver.Drive.DEADBAND.get())
            .withRotationalDeadband(Settings.Swerve.MAX_ANGULAR_VELOCITY * Settings.Driver.Turn.DEADBAND.get())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Vector2D velocity = Robot.isBlue() ? speed.get() : speed.get().mul(-1);
        swerve.setControl(drive.withVelocityX(velocity.y)
                .withVelocityY(-velocity.x)
                .withRotationalRate(turn.get())         
            );
    }
}