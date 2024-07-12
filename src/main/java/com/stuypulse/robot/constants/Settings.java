package com.stuypulse.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
  
    double DT = 1.0 / 50.0;

    double WIDTH = Units.inchesToMeters(32);
    double LENGTH = Units.inchesToMeters(36);
  
    // checks the current RIO's serial number to determine which robot is running
    public enum RobotType {
        // TODO: Add serial numbers from RIOs
        SELF_REINFORCED_VELVEETA_CHEESE_POLYPROPYLENE_GOOBER(""),
        SIM("");

        public final String serialNum;

        RobotType(String serialNum) {
            this.serialNum = serialNum;
        }

        public static RobotType fromString(String serialNum) {
            for (RobotType robot : RobotType.values()) {
                if (robot.serialNum.equals(serialNum.toUpperCase())) {
                    return robot;
                }
            }

            return RobotType.SIM;
        }
    }
  
    public interface Arm {
        // all constants need to be determined on robot and changed
        SmartNumber TELEOP_MAX_VELOCITY = new SmartNumber("Arm/Teleop Max Velocity (deg)", 315);
        SmartNumber TELEOP_MAX_ACCELERATION = new SmartNumber("Arm/Teleop Max Acceleration (deg)", 420);

        SmartNumber AUTON_MAX_VELOCITY = new SmartNumber("Arm/Auton Max Velocity (deg)", 360);
        SmartNumber AUTON_MAX_ACCELERATION = new SmartNumber("Arm/Auton Max Acceleration (deg)", 480);

        SmartNumber MAX_ANGLE = new SmartNumber("Arm/Max Angle (deg)", 100);
        SmartNumber MIN_ANGLE = new SmartNumber("Arm/Min Angle (deg)", -90 + 12.25);
        SmartNumber BUMP_SWITCH_DEBOUNCE_TIME = new SmartNumber("Arm/Bump Switch Debounce Time", 0.05);
        SmartNumber MAX_ANGLE_ERROR = new SmartNumber("Arm/Max Angle Error", 2);
        SmartNumber AMP_ANGLE = new SmartNumber("Arm/Amp Angle", 70);
        SmartNumber FERRY_ANGLE = new SmartNumber("Arm/Lob Ferry Angle", 60);
        SmartNumber PRE_CLIMB_ANGLE = new SmartNumber("Arm/Pre climb angle", 80);

        // feed angle is the furthest position the arm can be to still receive notes from the intake
        SmartNumber FEED_ANGLE = new SmartNumber("Arm/Feed Angle", MIN_ANGLE.getAsDouble() + 17);

        double MAX_WAIT_TO_REACH_TARGET = 2.0;

        // characterize and manually tune
        public interface PID {
            SmartNumber kP = new SmartNumber("Arm/kP", 0.45000);
            SmartNumber kI = new SmartNumber("Arm/kI", 0.0);
            SmartNumber kD = new SmartNumber("Arm/kD", 0.0);
        }

        public interface Feedforward {
            SmartNumber kS = new SmartNumber("Arm/kS", 0.44765);
            SmartNumber kV = new SmartNumber("Arm/kV", 0.10971); 
            SmartNumber kA = new SmartNumber("Arm/kA", 0.014801); 

            SmartNumber kG = new SmartNumber("Arm/kG", 0.14213); 
        }

        public interface Encoder {
            double GEAR_RATIO = 1.0 / (85 + 1.0/3); // 1 arm rotation (360 degrees) per 85.33 encoder ticks
        }
    }
  
    public interface Intake {
        double INTAKE_ACQUIRE_SPEED = 0.5;
        double INTAKE_DEACQUIRE_SPEED = 0.5;
        double FUNNEL_ACQUIRE = 1.0;
        double FUNNEL_DEACQUIRE = 1.0;

        double IRSensorTriggerTime = .03;
    }

    public interface Shooter {
        double FEEDER_SPEED = .22;

        double TARGET_RPM_THRESHOLD = 300;
        double MAX_WAIT_TO_REACH_TARGET = 1;
        
        ShooterSpeeds SPEAKER = new ShooterSpeeds(
            new SmartNumber("Shooter/Speaker RPM", 4875), 
            new SmartNumber("Shooter/Speaker RPM differential", 500)
        );

        ShooterSpeeds FERRY = new ShooterSpeeds(
            new SmartNumber("Shooter/Ferry RPM", 4875), 
            new SmartNumber("Shooter/Ferry RPM differential", 500)
        );

        SmartNumber HAS_NOTE_DEBOUNCE = new SmartNumber("Shooter/Has Note Debounce", 0.01);

        // left runs faster than right
        public interface LEFT {
            public interface PID {
                double kP = 0.00034711;
                double kI = 0;
                double kD = 0.0;
            }

            public interface FF {
                double kS = 0;
                double kV = 0.00018;
                double kA = 0;
            }
        }

        public interface RIGHT {
            public interface PID {
                double kP = 0.00034711;
                double kI = 0;
                double kD = 0.0;
            }

            public interface FF {
                double kS = 0;
                double kV = 0.00015;
                double kA = 0;
            }
        }
    }
    
    public interface Swerve {
        double WIDTH = Units.inchesToMeters(32.5);
        double LENGTH = Units.inchesToMeters(27.375);
        double CENTER_TO_INTAKE_FRONT = Units.inchesToMeters(13.0);

        double MAX_MODULE_SPEED = 4.9;
        double MAX_MODULE_ACCEL = 15.0;

        double MAX_LINEAR_VELOCITY = 15.0;
        double MAX_ANGULAR_VELOCITY = 12.0;

        double MODULE_VELOCITY_DEADBAND = 0.05;

        SmartNumber ALIGN_OMEGA_DEADBAND = new SmartNumber("Swerve/Align Omega Deadband", 0.05); // TODO: make 0.25 and test

        public interface Assist {
            SmartNumber ALIGN_MIN_SPEAKER_DIST = new SmartNumber("SwerveAssist/Minimum Distance to Speaker", 4); 

            // angle PID
            SmartNumber kP = new SmartNumber("SwerveAssist/kP", 6.0);
            SmartNumber kI = new SmartNumber("SwerveAssist/kI", 0.0);
            SmartNumber kD = new SmartNumber("SwerveAssist/kD", 0.0);

            double ANGLE_DERIV_RC = 0.05;
            double REDUCED_FF_DIST = 0.75;
        }

        public interface Motion {
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 4.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());

            PIDConstants XY = new PIDConstants(2.5, 0, 0.02);
            PIDConstants THETA = new PIDConstants(4, 0, 0.1);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / Swerve.Drive.L4;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/PID/kP", 7.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/PID/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/PID/kD", 0.05);

            SmartNumber kS = new SmartNumber("Swerve/Turn/FF/kS", 0.25582);
            SmartNumber kV = new SmartNumber("Swerve/Turn/FF/kV", 0.00205);
            SmartNumber kA = new SmartNumber("Swerve/Turn/FF/kA", 0.00020123);
            double kT = 1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp;

            SmartBoolean INVERTED = new SmartBoolean("Swerve/Turn/INVERTED", true);

            double TURN_REDUCTION = (150.0 / 7.0); // 21.4285714286
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Drive/PID/kP", 0.31399);
            SmartNumber kI = new SmartNumber("Swerve/Drive/PID/kI", 0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/PID/kD", 0);

            SmartNumber kS = new SmartNumber("Swerve/Drive/FF/kS", 0.00012288);
            SmartNumber kV = new SmartNumber("Swerve/Drive/FF/kV", 0.00012288);
            SmartNumber kA = new SmartNumber("Swerve/Drive/FF/kA", 0.0000259);

            boolean INVERTED = true;

            double L2 = ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0));
            double L3 = ((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0));
            double L4 = ((50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)); // 5.35714285714
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(38.144531);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-173.408203);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(24.609375);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(38.232422);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }
    }

    public interface Alignment {
        double DEBOUNCE_TIME = 0.05;

        SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance", 0.1);
        SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance", 0.1);
        SmartNumber ANGLE_TOLERANCE = new SmartNumber("Alignment/Angle Tolerance", 5);

        SmartNumber AMP_WALL_SETUP_DISTANCE = new SmartNumber("Alignment/Amp/Setup Pose Distance to Wall", Units.inchesToMeters(25.5));
        SmartNumber AMP_WALL_SCORE_DISTANCE = new SmartNumber("Alignment/Amp/Score Pose Distance to Wall", Units.inchesToMeters(22.5 - 1.75));

        SmartNumber TRAP_SETUP_DISTANCE = new SmartNumber("Alignment/Trap/Setup Pose Distance", Units.inchesToMeters(21.0));
        SmartNumber TRAP_CLIMB_DISTANCE = new SmartNumber("Alignment/Trap/Climb Distance", Units.inchesToMeters(18.0));

        SmartNumber INTO_CHAIN_SPEED = new SmartNumber("Alignment/Trap/Into Chain Speed", 0.25);

		double NOTE_TO_GOAL_TIME = 0.4;

        double MAX_ALIGNMENT_SPEED = 2.5;

        public interface Translation {
            SmartNumber kP = new SmartNumber("Alignment/Translation/kP", 6.0);
            SmartNumber kI = new SmartNumber("Alignment/Translation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Alignment/Translation/kD", 0.2);
        }

        public interface Rotation {
            SmartNumber kP = new SmartNumber("Alignment/Rotation/kP", 6.0);
            SmartNumber kI = new SmartNumber("Alignment/Rotation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Alignment/Rotation/kD", 0.4);
        }

        public interface Shoot {
            public interface Translation {
                SmartNumber kP = new SmartNumber("ShootAlign/Translation/kP", 7.5);
                SmartNumber kI = new SmartNumber("ShootAlign/Translation/kI", 0.0);
                SmartNumber kD = new SmartNumber("ShootAlign/Translation/kD", 0.7);
            }
    
            public interface Rotation {
                SmartNumber kP = new SmartNumber("ShootAlign/Rotation/kP", 6.0);
                SmartNumber kI = new SmartNumber("ShootAlign/Rotation/kI", 0.0);
                SmartNumber kD = new SmartNumber("ShootAlign/Rotation/kD", 0.4);
            }
        }
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.01);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_MODULE_SPEED);
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", 15);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Turn/Max Turning", 6.0);
        }
    }

    public interface NoteDetection {
        double X_ANGLE_RC = 0.05;

        SmartNumber HAS_NOTE_DEBOUNCE = new SmartNumber("Note Detection/Has Note Debounce", 0.2);

        SmartNumber THRESHOLD_X = new SmartNumber("Note Detection/X Threshold", 0.2);
        SmartNumber THRESHOLD_Y = new SmartNumber("Note Detection/Y Threshold", Units.inchesToMeters(2));
        SmartNumber THRESHOLD_ANGLE = new SmartNumber("Note Detection/Angle Threshold", 1);

        SmartNumber DRIVE_SPEED = new SmartNumber("Note Detection/Drive Speed", 1);

        SmartNumber INTAKE_THRESHOLD_DISTANCE = new SmartNumber("Note Detection/In Intake Path Distance", 0.9);

        double MAX_FULLY_IN_VIEW_ANGLE = 20;
        
        public interface Translation {
            SmartNumber kP = new SmartNumber("Note Detection/Translation/kP", 8.0);
            SmartNumber kI = new SmartNumber("Note Detection/Translation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Note Detection/Translation/kD", 0.0);
        }

        public interface Rotation {
            SmartNumber kP = new SmartNumber("Note Detection/Rotation/kP", 2.0);
            SmartNumber kI = new SmartNumber("Note Detection/Rotation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Note Detection/Rotation/kD", 0.0);
        }
    }

    public interface Buzz {
        double BUZZ_DURATION = 0.2;
        double BUZZ_INTENSITY = 1;
    }
}
