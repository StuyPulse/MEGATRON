/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

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

        double WIDTH = Units.inchesToMeters(0);
        double LENGTH = Units.inchesToMeters(0);

        double MAX_MODULE_SPEED = 0;
        double MAX_MODULE_ACCEL = 0;

        double MODULE_VELOCITY_DEADBAND = 0.0;

        SmartNumber ALIGN_OMEGA_DEADBAND = new SmartNumber("Swerve/Align Omega Deadband", 0.0);


        public interface Drive {
            public interface PID {
                SmartNumber kP = new SmartNumber("Drive kP", 0.0);
                SmartNumber kI = new SmartNumber("Drive kI", 0.0); 
                SmartNumber kD = new SmartNumber("Drive kD", 0.0);  
            }
            
        }

    public interface Encoder {
        public interface Drive {
            double WHEEL_DIAMETER = Units.inchesToMeters(0);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
            double GEAR_RATIO = 1.0 / 5.357;

            double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
        }

        public interface Turn {
            double POSITION_CONVERSION = 1;
            double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
    }
}
