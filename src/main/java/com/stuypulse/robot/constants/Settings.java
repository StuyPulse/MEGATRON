/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartNumber;

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

    public interface Shooter {
        double TARGET_RPM_THRESHOLD = 300;
        
        ShooterSpeeds SPEAKER = new ShooterSpeeds(
            new SmartNumber("Shooter/Speaker RPM", 4875), 
            new SmartNumber("Shooter/Speaker RPM differential", 500)
        );

        ShooterSpeeds AMPING = new ShooterSpeeds(-1000);

        ShooterSpeeds FERRY = new ShooterSpeeds(
            new SmartNumber("Shooter/Ferry RPM", 4875), 
            new SmartNumber("Shooter/Ferry RPM differential", 500)
        );

        SmartNumber HAS_NOTE_DEBOUNCE = new SmartNumber("Shooter/Has Note Debounce", 0.2);

        public interface PID {
            double kP = 0.00034711;
            double kI = 0;
            double kD = 0.0;
        }
    }

    public interface Feeder {
        
        double FEEDER_SPEED = 1.0;

        public interface Feedforward {
            double kS = 0.11873;
            double kV = 0.0017968;
            double kA = 0.00024169;
        }

        public interface PID {
            double kP = 0.00020863;
            double kI = 0.0;
            double kD = 0.0;
        }
    }
}
