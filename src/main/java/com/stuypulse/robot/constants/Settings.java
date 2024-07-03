/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
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
}
