package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Shoot;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveToShoot extends Command {

    private final SwerveDrive swerve;

    private final Controller distanceController;
    private final AnglePIDController angleController;
    private final IStream velocityError;

    private final IStream distanceToSpeaker;
    private final BStream isAligned;

    private final Number targetDistance;

    private double distanceTolerance;
    private double angleTolerance;
    private double velocityTolerance;

    
    public SwerveDriveToShoot() {
        this(Alignment.PODIUM_SHOT_DISTANCE);
    }

    public SwerveDriveToShoot(Number targetDistance) {
        this(targetDistance, Alignment.DEBOUNCE_TIME);
    }

    public SwerveDriveToShoot(Number targetDistance, double debounce) {
        this.targetDistance = targetDistance;

        swerve = SwerveDrive.getInstance();

        distanceController = new PIDController(Shoot.Translation.kP, Shoot.Translation.kI, Shoot.Translation.kD)
            .setOutputFilter(
                x -> -x,
                x -> MathUtil.clamp(x, -Alignment.MAX_ALIGNMENT_SPEED, +Alignment.MAX_ALIGNMENT_SPEED)
            );
        
        angleController = new AnglePIDController(Shoot.Rotation.kP, Shoot.Rotation.kI, Shoot.Rotation.kD);

        velocityError = IStream.create(distanceController::getError)
            .filtered(new Derivative())
            .filtered(new LowPassFilter(0.05))
            .filtered(x -> Math.abs(x));

        distanceToSpeaker = IStream.create(() -> getTranslationToSpeaker().getNorm())
            .filtered(new LowPassFilter(0.05));

        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Rising(debounce));
        
        distanceTolerance = 0.05;
        angleTolerance = Alignment.ANGLE_TOLERANCE.get();
        velocityTolerance = 0.1;

        addRequirements(swerve);

    }

    private double getTargetDistance() {
        return SLMath.clamp(targetDistance.doubleValue(), 1, 5);
    }

    public SwerveDriveToShoot withTolerance(double distanceTolerance, double angleTolerance) {
        this.distanceTolerance = distanceTolerance;
        this.angleTolerance = angleTolerance;
        return this;
    }
    
    public SwerveDriveToShoot withRotationConstants(double p, double i, double d) {
        angleController.setPID(p, i, d);
        return this;
    }

    private boolean isAligned() {
        return distanceController.isDone(distanceTolerance)
            && angleController.isDoneDegrees(angleTolerance)
            && velocityError.get() < velocityTolerance;
    }

    private Translation2d getTranslationToSpeaker() {
        return Field.getAllianceSpeakerPose().getTranslation().minus(swerve.getPose().getTranslation());
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("AutonAlignment", true);
    }

    @Override
    public void execute() {
        Rotation2d toSpeaker = getTranslationToSpeaker().getAngle();
        
        double speed = distanceController.update(getTargetDistance(), distanceToSpeaker.get());
        double rotation = angleController.update(
            Angle.fromRotation2d(toSpeaker).add(Angle.k180deg),
            Angle.fromRotation2d(swerve.getPose().getRotation()));

        Translation2d speeds = new Translation2d(
            speed,
            toSpeaker);

        if (Math.abs(rotation) < Swerve.ALIGN_OMEGA_DEADBAND.get())
            rotation = 0;

        swerve.setFieldRelativeSpeeds(
            new ChassisSpeeds(
                speeds.getX(),
                speeds.getY(),
                rotation));

        SmartDashboard.putNumber("Alignment/Velocity Error", velocityError.get());
        SmartDashboard.putNumber("Alignment/To Shoot Target Angle", toSpeaker.plus(Rotation2d.fromDegrees(180)).getDegrees());
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        SmartDashboard.putBoolean("AutonAlignment", false);
    }

    // NEED TO CHANGE SO THAT ALLOWED SHOOT DISTANCE IS ALL THE WAY FROM ALLIANCE WING LINE AND NOT AROUND PDOIUM

}
