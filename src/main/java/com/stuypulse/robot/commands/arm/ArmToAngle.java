package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmToAngle extends InstantCommand{
    
    public static Command untilDone(double angle) {
        return untilDone(angle, Settings.Arm.MAX_ANGLE_ERROR.doubleValue());
    }

    public static Command untilDone(double angle, double epsilon) {
        return new ArmToAngle(angle)
            .andThen(new WaitUntilCommand(() -> Arm.getInstance().atTargetDegrees(epsilon)));
    }

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
}
