package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmToFerry extends InstantCommand{
    
    private final Arm arm;
    private final StopWatch stopWatch;
    private double lastClick;

    public ArmToFerry() {
        arm = Arm.getInstance();
        stopWatch = new StopWatch();
        lastClick = 0;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (stopWatch.getTime() - lastClick < Settings.Driver.DOUBLE_CLICK_TIME_BETWEEN_CLICKS) {
            arm.setState(Arm.State.LOB_FERRY);
        }
        else {
            arm.setState(Arm.State.LOW_FERRY);
        }
        lastClick = stopWatch.getTime();
    }

}
