package com.stuypulse.robot.commands.swerve.noteAlignment;

import static com.stuypulse.robot.constants.Settings.Alignment.*;
import static com.stuypulse.robot.constants.Settings.NoteDetection.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.NoteDetection;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.util.HolonomicController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveToNote extends Command {

    private final SwerveDrive swerve;
    private final NoteVision vision;
    private final Intake intake;

    private final SwerveRequest.FieldCentric drive;

    private final VStream speed;

    private final HolonomicController controller;
    private final BStream aligned;

    public SwerveDriveDriveToNote(Gamepad driver) {
        this.swerve = SwerveDrive.getInstance();
        this.vision = NoteVision.getInstance();
        this.intake = Intake.getInstance();

        drive = new SwerveRequest.FieldCentric()
            .withDeadband(Settings.Swerve.MAX_LINEAR_VELOCITY * Settings.Driver.Drive.DEADBAND.get())
            .withRotationalDeadband(Settings.Swerve.MAX_ANGULAR_VELOCITY * Settings.Driver.Turn.DEADBAND.get())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1),
                x -> x.pow(Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL.get()),
                new VLowPassFilter(Drive.RC.get()));

        controller = new HolonomicController(
            new PIDController(NoteDetection.Translation.kP, NoteDetection.Translation.kI, NoteDetection.Translation.kD),
            new PIDController(NoteDetection.Translation.kP, NoteDetection.Translation.kI, NoteDetection.Translation.kD),
            new AnglePIDController(NoteDetection.Rotation.kP, NoteDetection.Rotation.kI, NoteDetection.Rotation.kD));

        SmartDashboard.putData("Note Detection/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));

        addRequirements(swerve);
    }

    private boolean isAligned() {
        return controller.isDone(THRESHOLD_X.get(), THRESHOLD_Y.get(), THRESHOLD_ANGLE.get());
    }

    @Override
    public void execute() {
        Translation2d targetTranslation = swerve.getPose()
            .getTranslation().plus(
                new Translation2d(Settings.CENTER_TO_FRONT_OF_INTAKE, 0)
                    .rotateBy(swerve.getPose().getRotation()));

        Rotation2d targetRotation = vision.getEstimatedNotePose().minus(targetTranslation).getAngle();

        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

        if (vision.hasNoteData()) {
            ChassisSpeeds speeds = controller.update(targetPose, swerve.getPose());

            if (vision.withinIntakePath())
                speeds.omegaRadiansPerSecond = Math.signum(speeds.omegaRadiansPerSecond) * Math.toRadians(5.0);

            // drive to note
            swerve.setControl(drive.withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)      
            );
        } else {
            swerve.setControl(drive.withVelocityX(speed.get().y)
                .withVelocityY(-speed.get().x)
                .withRotationalRate(0)         
            );
        }

        SmartDashboard.putBoolean("Note Detection/Is Aligned", aligned.get());
    }

    @Override
    public boolean isFinished() {
        return aligned.get() || intake.hasNote();
    }
}
