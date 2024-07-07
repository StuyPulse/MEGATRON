package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmToAngle extends Command{

    private final Arm arm;
    private final double angle;
    private final double epsilon;

    public ArmToAngle(double angle, double epsilon) {
        arm = Arm.getInstance();
        this.angle = angle;
        this.epsilon = epsilon;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetDegrees(angle);
    }

    @Override
    public boolean isFinished() {
        return arm.atTargetDegrees(epsilon);
    }
}
