/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Intake {
        double INTAKE_ACQUIRE_SPEED = 1.0;
        double INTAKE_DEACQUIRE_SPEED = 1.0;
        double FUNNEL_ACQUIRE = 1.0;
        double FUNNEL_DEACQUIRE = 1.0;

        double IRSensorTriggerTime = .03;
    }
}
