/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    
    double DT = 1.0/50.0; // IZZI CONSTANT
    public interface SWERVE {
        // TODO: Change and tune these values. Change to SmartNumber if necessary
        double WIDTH = Units.inchesToMeters(0.0);
        double LENGTH = Units.inchesToMeters(0.0);

        double MAX_MODULE_SPEED = 0.0;
        double MAX_MODULE_ACCEL = 0.0;

        double MODULE_VELOCITY_DEADBAND = 0.0;

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
            boolean INVERTED = true;

            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kT = 0.0;    // Feedforward for turning (torque)

            double TURN = 150.0 / 7.0; // 21.4285714286

            double MAX_FORWARD_TORQUE = 40.0;
            double MIN_REVERSE_TORQUE = -40.0;
        }

        public interface Drive {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;

            double L4 = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0); // 5.35714285714

            double MAX_FORWARD_TORQUE = 80.0;
            double MIN_REVERSE_TORQUE = -80.0;
            double TORQUE_RAMP_RATE = 0.02;
        }

        // TODO: Add Translation2D MODULE_OFFSET???

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
            boolean INVERTED = true;
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
            boolean INVERTED = true;
        }
        
        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
            boolean INVERTED = true;
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
            boolean INVERTED = true;
        }
    }
}
