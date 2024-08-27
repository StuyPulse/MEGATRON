package com.stuypulse.robot;

import com.ctre.phoenix6.Utils;
import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.arm.ArmToAmp;
import com.stuypulse.robot.commands.arm.ArmToClimbing;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToLobFerry;
import com.stuypulse.robot.commands.arm.ArmToLowFerry;
import com.stuypulse.robot.commands.arm.ArmToPreClimb;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeFeed;
import com.stuypulse.robot.commands.intake.IntakeShoot;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.leds.LEDDefaultMode;
import com.stuypulse.robot.commands.leds.LEDReset;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntake;
import com.stuypulse.robot.commands.shooter.ShooterFeederDeacquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterManualIntake;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveRobotRelative;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedAmp;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedLobFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedLowFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedManualLobFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedManualLowFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedSpeaker;
import com.stuypulse.robot.commands.swerve.noteAlignment.SwerveDriveDriveToNote;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.util.SLColor;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.leds.instructions.LEDPulseColor;
import com.stuypulse.robot.subsystems.leds.instructions.LEDRainbow;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public final LEDController leds = LEDController.getInstance();

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
        intake.setDefaultCommand(new IntakeStop());
        leds.setDefaultCommand(new LEDDefaultMode());
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
            // .whileTrue(new SwerveDriveDriveToNote(driver))
            .whileTrue(new IntakeAcquire()
                .deadlineWith(new LEDSet(LEDInstructions.FIELD_RELATIVE_INTAKING))
                .andThen(new BuzzController(driver))
            );
        
        // intake robot relative
        driver.getLeftTriggerButton()
            .onTrue(new ArmToFeed())
            .whileTrue(new IntakeAcquire()
                .deadlineWith(new LEDSet(LEDInstructions.ROBOT_RELATIVE_INTAKING))
                .andThen(new BuzzController(driver))
            )
            .whileTrue(new SwerveDriveDriveRobotRelative(driver));
        
        // deacquire
        driver.getDPadLeft()
            .whileTrue(new IntakeDeacquire())
            .whileTrue(new LEDSet(LEDInstructions.DEACQUIRING));
        
        // speaker align and score 
        // score amp
        driver.getRightBumper()
            .whileTrue(new ConditionalCommand(
                new ArmWaitUntilAtTarget()
                    .withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                    .andThen(new ShooterFeederDeacquire().alongWith(new LEDSet(LEDInstructions.AMP_SCORE))),
                new SwerveDriveDriveAlignedSpeaker(driver)
                    .alongWith(new ArmToSpeaker().alongWith(new ShooterSetRPM(Settings.Shooter.SPEAKER))
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToSpeaker()))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                            )
                    )
                    .alongWith(new LEDSet(LEDInstructions.SPEAKER_ALIGN)),
                () -> Arm.getInstance().getState() == Arm.State.AMP))
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));

        // lob ferry align and shoot
        driver.getLeftStickButton()
            .whileTrue(new SwerveDriveDriveAlignedLobFerry(driver)
                    .alongWith(new ArmToLobFerry().alongWith(new ShooterSetRPM(() -> shooter.getFerrySpeeds()))
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToLobFerry()))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                            )
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOB_FERRY_ALIGN))
            )
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));


        // low ferry align and shoot
        driver.getRightStickButton()
            .whileTrue(new SwerveDriveDriveAlignedLowFerry(driver)
                    .alongWith(new ArmToLowFerry().alongWith(new ShooterSetRPM(() -> shooter.getFerrySpeeds()))
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToLowFerry()))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                            )
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOW_FERRY_ALIGN))
            )
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));

        // arm to amp
        driver.getLeftBumper().onTrue(new ArmToAmp());

        // manual speaker at subwoofer
        driver.getRightMenuButton()
            .whileTrue(new ArmToSubwooferShot().alongWith(new ShooterSetRPM(Settings.Shooter.SPEAKER))
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                            )
                        )
            .whileTrue(new LEDSet(LEDInstructions.SPEAKER_MANUAL))
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));
        
        // manual lob ferry
        driver.getTopButton()
            .whileTrue(new SwerveDriveDriveAlignedManualLobFerry(driver)
                    .alongWith(new ArmToLobFerry().alongWith(new ShooterSetRPM(() -> shooter.getFerrySpeeds()))
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToManualLobFerry()))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                            )
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOB_FERRY_ALIGN_MANUAL))
            )
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));

        // manual low ferry
        driver.getLeftButton()
            .whileTrue(new SwerveDriveDriveAlignedManualLowFerry(driver)
                    .alongWith(new ArmToLowFerry().alongWith(new ShooterSetRPM(() -> shooter.getFerrySpeeds()))
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToManualLowFerry()))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                        )
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOW_FERRY_ALIGN_MANUAL))
            )
            .onFalse(new ConditionalCommand(
                new ShooterFeederStop(), 
                new ShooterStop(), 
                () -> Settings.Shooter.ALWAYS_KEEP_AT_SPEED));
    }

    private void configureOperatorBindings() {

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

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "Do Nothing";
        }
        
        return autonChooser.getSelected().getName();
    }
}
