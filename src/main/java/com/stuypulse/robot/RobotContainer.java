package com.stuypulse.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.commands.arm.ArmToAmp;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToFerry;
import com.stuypulse.robot.commands.arm.ArmToPreClimb;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToStow;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntake;
import com.stuypulse.robot.commands.shooter.ShooterAutoShoot;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
// import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
// import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedSpeaker;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.TunerConstants;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
// import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
// import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final NoteVision noteVision = NoteVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    
    public final Intake intake = Intake.getInstance();
    // public final Shooter shooter = Shooter.getInstance();
    // public final Arm arm = Arm.getInstance();
    public final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;

    // Swerve Drive Stuff
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(Settings.Swerve.MAX_LINEAR_VELOCITY * 0.1).withRotationalDeadband(Settings.Swerve.MAX_ANGULAR_VELOCITY * 0.1) // 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Settings.Swerve.MAX_LINEAR_VELOCITY);

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        // swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        
        swerve.setDefaultCommand( 
        swerve.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * Settings.Swerve.MAX_LINEAR_VELOCITY) // Drive forward with
                                                                                            // negative Y (forward)
                .withVelocityY(-driver.getLeftX() * Settings.Swerve.MAX_LINEAR_VELOCITY) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * Settings.Swerve.MAX_ANGULAR_VELOCITY) // Drive counterclockwise with negative X (left)
            ));

            driver.getBottomButton().whileTrue(swerve.applyRequest(() -> brake));
            driver.getRightButton().whileTrue(swerve
            .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driver.getLeftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));

        if (Utils.isSimulation()) {
            swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        swerve.registerTelemetry(logger::telemeterize);
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        // driver.getLeftTriggerButton().whileTrue(new IntakeAcquire());
        
        // driver.getLeftBumper()
        //     .whileTrue(new WaitUntilCommand(() -> !shooter.hasNote())
        //     .andThen(new ArmSetState(Arm.State.FEED)
        //         .andThen(new WaitUntilCommand(intake::hasNote).alongWith(new ArmWaitUntilAtTarget()))
        //         .andThen(new ShooterAcquireFromIntake())
        //     )
        // );
        
        // driver.getDPadLeft()
        //     .whileTrue(new IntakeDeacquire())
        //     .onFalse(new IntakeStop());

        // driver.getRightTriggerButton()
        //     .whileTrue(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
        //         .andThen(new ShooterAutoShoot())
        //     );
        
        // driver.getRightBumper()
        //     .onTrue(new SwerveDriveDriveAlignedSpeaker(driver));
        
        // driver.getTopButton().onTrue(new ArmToSpeaker());
        // driver.getLeftButton().onTrue(new ArmToAmp());
        // driver.getRightButton().onTrue(new ArmToFerry());
        // driver.getBottomButton().onTrue(new ArmToFeed());
        
        // driver.getDPadUp().onTrue(new ArmToPreClimb());
        // driver.getDPadDown().onTrue(new ArmToStow());

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
