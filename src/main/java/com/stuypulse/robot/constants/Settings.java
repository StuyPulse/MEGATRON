/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public enum Robot {
        OFFSEASON
    }

    Robot ROBOT = Robot.OFFSEASON;

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
        //feed angle is the furthest position the arm can be to still receive notes from the intake
        SmartNumber FEED_ANGLE = new SmartNumber("Arm/Feed Angle", -87);

        public interface PID {
            SmartNumber kP = new SmartNumber("Arm/kP", 1.3);
            SmartNumber kI = new SmartNumber("Arm/kI", 0);
            SmartNumber kD = new SmartNumber("Arm/kD", 0.25);
        }

        public interface Feedforward {
            SmartNumber kS = new SmartNumber("Arm/kS", 0.061);
            SmartNumber kV = new SmartNumber("Arm/kV", 1.2);
            SmartNumber kA = new SmartNumber("Arm/kA", 0.038097);
            SmartNumber kGEmpty = new SmartNumber("Arm/kG Empty", 0.7);

            SmartNumber kGNote = new SmartNumber("Arm/kG Note", 0.7); // TODO: determine kGNote
        }

        public interface Encoder {
            double GEAR_RATIO = 1.0 / 85.33;
        }
    }
}
