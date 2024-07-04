/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Shooter {
        double SHOOT_TIME_DEBOUNCE = 0.4;

        double MOMENT_OF_INERTIA = 0.01;

        double TELEOP_SHOOTER_STARTUP_DELAY = 0.5;

        // MAX RPM
        // LEFT/RIGHT: 5900
        // FEEDER: 3100
        ShooterSpeeds PODIUM_SHOT = new ShooterSpeeds(
            new SmartNumber("Shooter/Podium Shooter RPM", 5500),
            500,
            new SmartNumber("Shooter/Podium Feeder RPM", 3000));

        ShooterSpeeds HANDOFF = new ShooterSpeeds(3000, 3000);
        
        ShooterSpeeds REVERSE = new ShooterSpeeds(-3000, -3000);

        ShooterSpeeds AMPING = new ShooterSpeeds(-3000, 3100);

        ShooterSpeeds FEEDER_STOP = new ShooterSpeeds(0,0);

        ShooterSpeeds FEEDER_GET = new ShooterSpeeds(0, 3100);

        ShooterSpeeds FERRY = new ShooterSpeeds(
            new SmartNumber("Shooter/Ferry Shooter RPM", 6000), // 5500
            500,
            new SmartNumber("Shooter/Ferry Feeder RPM", 3500)); // 3000

        ShooterSpeeds WING_FERRY = new ShooterSpeeds(2000, 2500);

        double AT_RPM_EPSILON = 200;

        SmartNumber HAS_NOTE_DEBOUNCE = new SmartNumber("Shooter/Has Note Debounce", 0.2);
        SmartNumber RPM_CHANGE_RC = new SmartNumber("Shooter/RPM Change RC", 0.2);
        double RPM_CHANGE_DIP_THRESHOLD = 300;

        public interface PID {
            double kP = 0.00034711;
            double kI = 0;
            double kD = 0.0;
        }
    }

    public interface Feeder {
        
        double FEEDER_ON = 1.0;

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

    public interface Motors {

        public enum StatusFrame {
        APPLIED_OUTPUT_FAULTS,
        MOTOR_VEL_VOLTS_AMPS,
        MOTOR_POSITION,
        ANALOG_SENSOR,
        ALTERNATE_ENCODER,
        ABS_ENCODER_POSIITION,
        ABS_ENCODER_VELOCITY
        }
        public static void disableStatusFrames(CANSparkBase motor, StatusFrame... ids) {
        
        final int kDisableStatusFrame = 500;

        for (StatusFrame id : ids) {
            motor.setPeriodicFramePeriod(PeriodicFrame.fromId(id.ordinal()), kDisableStatusFrame);
        }

        }
    }
}
