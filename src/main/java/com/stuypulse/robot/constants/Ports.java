/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Intake {
        int FUNNEL_LEFT = 21; 
        int FUNNEL_RIGHT = 22;
        int INTAKE_MOTOR = 20;

        int IRSensor = 0; // TO DO: FIND IR SENSOR PORT
    }

}
