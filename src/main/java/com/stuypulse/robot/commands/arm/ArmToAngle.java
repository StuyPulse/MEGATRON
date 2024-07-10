package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmToAngle extends Command{

    private final Arm arm;
    private final double angle;

    public ArmToAngle(double angle) {
        arm = Arm.getInstance();
        this.angle = angle;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return arm.atTargetDegrees(Settings.Arm.MAX_ANGLE_ERROR.getAsDouble());
    }
}
