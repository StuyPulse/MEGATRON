package com.stuypulse.robot;

import com.ctre.phoenix6.Utils;
import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.arm.ArmToAmp;
import com.stuypulse.robot.commands.arm.ArmToClimbing;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToFerry;
import com.stuypulse.robot.commands.arm.ArmToPreClimb;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToStow;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntake;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterScoreAmp;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveRobotRelative;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToChain;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveToClimb;
import com.stuypulse.robot.commands.swerve.SwerveDriveXMode;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedAmp;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedLowFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedSpeakerHigh;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedSpeakerLow;
import com.stuypulse.robot.commands.swerve.driveAndShoot.SwerveDriveDriveAndLobFerry;
import com.stuypulse.robot.commands.swerve.driveAndShoot.SwerveDriveDriveAndLobFerryManual;
import com.stuypulse.robot.commands.swerve.driveAndShoot.SwerveDriveDriveAndLowFerry;
import com.stuypulse.robot.commands.swerve.driveAndShoot.SwerveDriveDriveAndLowFerryManual;
import com.stuypulse.robot.commands.swerve.driveAndShoot.SwerveDriveDriveAndScoreSpeaker;
import com.stuypulse.robot.commands.swerve.noteAlignment.SwerveDriveDriveToNote;
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
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
        configureOperatorBindings();
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        driver.getDPadUp().onTrue(new SwerveDriveSeedFieldRelative());

        // intake field relative
        driver.getRightTriggerButton()
            .onTrue(new ArmToFeed())
            .whileTrue(new SwerveDriveDriveToNote(driver))
            .whileTrue(new IntakeAcquire()
                .andThen(new BuzzController(driver))
            );
        
        // intake robot relative
        driver.getLeftTriggerButton()
            .onTrue(new ArmToFeed())
            .whileTrue(new IntakeAcquire()
                .andThen(new BuzzController(driver))
            )
            .whileTrue(new SwerveDriveDriveRobotRelative(driver));
        
        // deacquire
        driver.getDPadLeft()
            .whileTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());
        
        // speaker align and score 
        driver.getRightBumper().whileTrue(new SwerveDriveDriveAndScoreSpeaker(driver));

        // ferry align and shoot
        // move to back of controller
        driver.getDPadRight().whileTrue(new SwerveDriveDriveAndLobFerry(driver));
        driver.getDPadDown().whileTrue(new SwerveDriveDriveAndLowFerry(driver));

        // arm to amp and alignment
        driver.getLeftBumper()
            .onTrue(new ArmToAmp())
            .onTrue(new SwerveDriveDriveAlignedAmp(driver));

        // manual speaker at subwoofer
        // score amp
        // rebind to a button on the back later
        driver.getRightMenuButton()
            .whileTrue(new ConditionalCommand(
                new ShooterScoreAmp(), 
                new ArmToSubwooferShot()
                    .andThen(new ShooterScoreSpeaker()), 
                () -> Arm.getInstance().getState() == Arm.State.AMP))
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));
        
        // manual ferry
        driver.getTopButton().whileTrue(new SwerveDriveDriveAndLobFerryManual(driver));
        driver.getLeftButton().whileTrue(new SwerveDriveDriveAndLowFerryManual(driver));
        
        // climbing
        driver.getRightButton().onTrue(new ArmToPreClimb());
        driver.getBottomButton().onTrue(new ArmToClimbing());
    }

    private void configureOperatorBindings() {
        operator.getLeftTriggerButton().whileTrue(new IntakeDeacquire());
        operator.getRightTriggerButton().whileTrue(new IntakeAcquire());
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
