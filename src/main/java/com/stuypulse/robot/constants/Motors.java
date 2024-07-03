/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkMax.IdleMode;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
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

    /** Classes to store all of the values a motor needs */
    public interface Swerve {
        CANSparkMaxConfig DRIVE_CONFIG = new CANSparkMaxConfig(true, IdleMode.kBrake, 60, 0.1);
        CANSparkMaxConfig TURN_CONFIG = new CANSparkMaxConfig(false, IdleMode.kBrake, 80);
    }

    public static class TalonSRXConfig {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final int PEAK_CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public TalonSRXConfig(
                boolean inverted,
                NeutralMode neutralMode,
                int peakCurrentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.PEAK_CURRENT_LIMIT_AMPS = peakCurrentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode, int peakCurrentLimitAmps) {
            this(inverted, neutralMode, peakCurrentLimitAmps, 0.0);
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode) {
            this(inverted, neutralMode, 80);
        }

        public void configure(WPI_TalonSRX motor) {
            motor.setInverted(INVERTED);
            motor.setNeutralMode(NEUTRAL_MODE);
            motor.configContinuousCurrentLimit(PEAK_CURRENT_LIMIT_AMPS - 10, 0);
            motor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT_AMPS, 0);
            motor.configPeakCurrentDuration(100, 0);
            motor.enableCurrentLimit(true);
            motor.configOpenloopRamp(OPEN_LOOP_RAMP_RATE);
        }
    }

    public static class VictorSPXConfig {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final double OPEN_LOOP_RAMP_RATE;

        public VictorSPXConfig(
                boolean inverted,
                NeutralMode neutralMode,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public VictorSPXConfig(boolean inverted, NeutralMode neutralMode) {
            this(inverted, neutralMode, 0.0);
        }

        public void configure(WPI_VictorSPX motor) {
            motor.setInverted(INVERTED);
            motor.setNeutralMode(NEUTRAL_MODE);
            motor.configOpenloopRamp(OPEN_LOOP_RAMP_RATE);
        }
    }

    public static class CANSparkMaxConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public CANSparkMaxConfig(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 80);
        }

        public void configure(CANSparkMax motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
        }

        public void configureAsFollower(CANSparkMax motor, CANSparkMax follows) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.follow(follows);
            motor.burnFlash();
         }
     }
}
