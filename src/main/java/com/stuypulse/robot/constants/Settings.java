/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

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
        TUMBLER(""),
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
        SmartNumber MIN_ANGLE = new SmartNumber("Arm/Min Angle (deg)", -90);
        SmartNumber BUMP_SWITCH_DEBOUNCE_TIME = new SmartNumber("Arm/Bump Switch Debounce Time", 0.1);
        SmartNumber MAX_ANGLE_ERROR = new SmartNumber("Arm/Max Angle Error", 1);
        SmartNumber SPEAKER_ANGLE = new SmartNumber("Arm/Speaker Angle", -70);
        SmartNumber AMP_ANGLE = new SmartNumber("Arm/Amp Angle", 80);
        SmartNumber FERRY_ANGLE = new SmartNumber("Arm/Ferry Angle", -80);

        // feed angle is the furthest position the arm can be to still receive notes from the intake
        SmartNumber FEED_ANGLE = new SmartNumber("Arm/Feed Angle", -87);

        // characterize and manually tune
        public interface PID {
            SmartNumber kP = new SmartNumber("Arm/kP", 1.3);
            SmartNumber kI = new SmartNumber("Arm/kI", 0);
            SmartNumber kD = new SmartNumber("Arm/kD", 0.25);
        }

        public interface Feedforward {
            SmartNumber kS = new SmartNumber("Arm/kS", 0.061);
            SmartNumber kV = new SmartNumber("Arm/kV", 1.2);
            SmartNumber kA = new SmartNumber("Arm/kA", 0.038097);

            SmartNumber kGEmpty = new SmartNumber("Arm/kG Empty", 0.7); // TODO: determine kG
            SmartNumber kGNote = new SmartNumber("Arm/kG Note", 0.7);
        }

        public interface Encoder {
            double GEAR_RATIO = 1.0 / (85 + 1.0/3); // 1 arm rotation (360 degrees) per 85.33 encoder ticks
        }
    }
  
    public interface Intake {
        double INTAKE_ACQUIRE_SPEED = 1.0;
        double INTAKE_DEACQUIRE_SPEED = 1.0;
        double FUNNEL_ACQUIRE = 1.0;
        double FUNNEL_DEACQUIRE = 1.0;

        double IRSensorTriggerTime = .03;
    }

    public interface Shooter {
        double TARGET_RPM_THRESHOLD = 300;
        
        ShooterSpeeds SPEAKER = new ShooterSpeeds(
            new SmartNumber("Shooter/Speaker RPM", 4875), 
            new SmartNumber("Shooter/Speaker RPM differential", 500)
        );

        ShooterSpeeds AMPING = new ShooterSpeeds(-1000, -1);

        ShooterSpeeds FERRY = new ShooterSpeeds(
            new SmartNumber("Shooter/Ferry RPM", 4875), 
            new SmartNumber("Shooter/Ferry RPM differential", 500)
        );

        SmartNumber HAS_NOTE_DEBOUNCE = new SmartNumber("Shooter/Has Note Debounce", 0.2);

        public interface LEFT {
            public interface PID {
                double kP = 0.00034711;
                double kI = 0;
                double kD = 0.0;
            }

            public interface FF {
                double kS = 0;
                double kV = 0;
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
                double kV = 0;
                double kA = 0;
            }
        }

        double FEEDER_SPEED = 1.0;
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

        // public interface Assist {
        //     SmartNumber ALIGN_MIN_SPEAKER_DIST = new SmartNumber("SwerveAssist/Minimum Distance to Speaker", 4); //change

        //     double BUZZ_INTENSITY = 1;

        //     // angle PID
        //     SmartNumber kP = new SmartNumber("SwerveAssist/kP", 6.0);
        //     SmartNumber kI = new SmartNumber("SwerveAssist/kI", 0.0);
        //     SmartNumber kD = new SmartNumber("SwerveAssist/kD", 0.0);

        //     double ANGLE_DERIV_RC = 0.05;
        //     double REDUCED_FF_DIST = 0.75;
        // }

        // // TODO: Tune these values
        // public interface Motion {
        //     SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 3.0);
        //     SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 4.0);
        //     SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
        //     SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

        //     PathConstraints DEFAULT_CONSTRAINTS =
        //         new PathConstraints(
        //             MAX_VELOCITY.get(),
        //             MAX_ACCELERATION.get(),
        //             MAX_ANGULAR_VELOCITY.get(),
        //             MAX_ANGULAR_ACCELERATION.get());

        //     PIDConstants XY = new PIDConstants(2.5, 0, 0.02);
        //     PIDConstants THETA = new PIDConstants(4, 0, 0.1);
        // }

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

    public interface Driver {
        public interface Drive {
            double BUZZ_DURATION = 0.2;
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
}
