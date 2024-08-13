/************************ PROJECT IZZI *************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultMode extends Command {

    private final LEDController leds;
    private final SwerveDrive swerve;
    private final Intake intake;
    private final Shooter shooter;
    private final Arm arm;

    public LEDDefaultMode() {
        leds = LEDController.getInstance();
        swerve = SwerveDrive.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
        arm = Arm.getInstance();

        addRequirements(leds);
    }

    private LEDInstruction getInstruction() {

        switch (arm.getState()) {
            case PRE_CLIMB:
                return LEDInstructions.ARM_PRECLIMB;
            case CLIMBING:
                return LEDInstructions.ARM_POSTCLIMB;
            case AMP:
                return LEDInstructions.AMP_WITHOUT_ALIGN;
            default:
                break;
        }

        if (intake.hasNote() || shooter.hasNote()) {
            return LEDInstructions.CONTAINS_NOTE;
        }
        
        return LEDInstructions.DEFAULT;
    }

    @Override
    public void execute() {
        leds.runLEDInstruction(getInstruction());
    }
}