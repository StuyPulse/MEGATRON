/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Swerve {
        // TODO: Change some values to SmartNumber


        // TODO: Change Encoder constants if necessary
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
        }

        public interface Drive {
            boolean INVERTED = true;

            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;

            double L4 = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0); // 5.35714285714
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
        }
        
        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
        }
    }
}
