package com.stuypulse.robot;

import com.ctre.phoenix6.Utils;
import com.stuypulse.robot.commands.BuzzController;
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
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignedFerryAndShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignedSpeakerAndShoot;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedFerry;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedSpeaker;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final NoteVision noteVision = NoteVision.getInstance();
    
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Arm arm = Arm.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    private final Telemetry logger = new Telemetry();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        if (Utils.isSimulation()) {
            swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        swerve.registerTelemetry(logger::telemeterize);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getRightMenuButton().whileTrue(new SwerveDriveXMode());
        driver.getLeftMenuButton().onTrue(new SwerveDriveSeedFieldRelative());

        driver.getLeftTriggerButton()
            .whileTrue(new IntakeAcquire()
                .andThen(new BuzzController(driver))
            );
        
        // driver.getLeftBumper()
        //     .whileTrue(new WaitUntilCommand(() -> !shooter.hasNote())
        //     .andThen(new ArmToFeed()
        //         .andThen(new WaitUntilCommand(intake::hasNote).alongWith(new ArmWaitUntilAtTarget()))
        //         .andThen(new ShooterAcquireFromIntake())
        //         .andThen(new BuzzController(driver))
        //     )
        // );

        // driver.getLeftTriggerButton()
        //     .whileTrue(new ArmToFeed().onlyIf(() -> !shooter.hasNote()))
        //     .whileTrue(new IntakeAcquire()
        //         .andThen(new BuzzController(driver))
        //         .andThen(new WaitUntilCommand(() -> Arm.getInstance().getState() == Arm.State.FEED))
        //         .andThen(new ArmWaitUntilAtTarget())
        //         .andThen(new ShooterAcquireFromIntake().onlyIf(() -> !shooter.hasNote()))
        //         .andThen(new BuzzController(driver)))
        //     .onFalse(new IntakeStop());
        
        driver.getLeftBumper()
            .whileTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());

        driver.getRightTriggerButton()
            .whileTrue(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                .andThen(new ShooterAutoShoot())
            );
        
        driver.getRightBumper()
            .onTrue(new SwerveDriveDriveAlignedSpeaker(driver));
        
        driver.getTopButton()
            .onTrue(new ArmToSpeaker());
            // .onTrue(new SwerveDriveDriveAlignedSpeaker(driver));
        driver.getLeftButton().onTrue(new ArmToAmp());
        driver.getRightButton()
            .onTrue(new ArmToFerry());
            // .onTrue(new SwerveDriveAlignedFerry(driver));
        driver.getBottomButton().onTrue(new ArmToFeed());
        
        driver.getDPadUp().onTrue(new ArmToPreClimb());
        driver.getDPadDown().onTrue(new ArmToStow());

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
