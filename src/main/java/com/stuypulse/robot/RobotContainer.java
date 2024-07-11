package com.stuypulse.robot;

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
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveAlignedSpeaker;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.odometry.Odometry;

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
    public final Shooter shooter = Shooter.getInstance();
    public final Arm arm = Arm.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();

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
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getLeftTriggerButton()
            .whileTrue(new IntakeAcquire())
            .whileTrue(new WaitUntilCommand(() -> !shooter.hasNote())
                .andThen(new ArmSetState(Arm.State.FEED)
                    .andThen(new WaitUntilCommand(intake::hasNote))
                    .andThen(new ShooterAcquireFromIntake())
                )
            );
        
        driver.getLeftBumper()
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());

        driver.getRightTriggerButton()
            .whileTrue(new ArmWaitUntilAtTarget().alongWith(new ShooterWaitForTarget())
                .andThen(new ShooterAutoShoot())
            );
        
        driver.getRightBumper()
            .onTrue(new SwerveDriveDriveAlignedSpeaker(driver));
        
        driver.getTopButton().onTrue(new ArmToSpeaker());
        driver.getLeftButton().onTrue(new ArmToAmp());
        driver.getRightButton().onTrue(new ArmToFerry());
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
