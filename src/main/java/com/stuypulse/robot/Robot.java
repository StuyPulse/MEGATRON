package com.stuypulse.robot;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.intake.IntakeShoot;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntake;
import com.stuypulse.robot.commands.shooter.ShooterManualIntake;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    
    public static final RobotType ROBOT;

    static {
        if (Robot.isSimulation())
            ROBOT = RobotType.SIM;
        else
            ROBOT = RobotType.fromString(System.getenv("serialnum"));
    }

    private RobotContainer robot;
    private CommandScheduler scheduler;
    private Command auto;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStar());

        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();

        robot = new RobotContainer();

        SmartDashboard.putString("Robot State", "DISABLED");
        SmartDashboard.putString("Robot", ROBOT.name());

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // reload whitelist in case of alliance change
        CommandScheduler.getInstance().schedule(new VisionReloadWhiteList());
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }

        SmartDashboard.putString("Robot State", "AUTON");

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        if (auto != null) {
            auto.cancel();
        }
        robot.configureAutomaticCommandScheduling();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
