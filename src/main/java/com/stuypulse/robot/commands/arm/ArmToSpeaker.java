package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmToSpeaker extends InstantCommand{
    
    private final Arm arm;
    private final StopWatch stopWatch;
    private double lastClick;

    public ArmToSpeaker() {
        arm = Arm.getInstance();
        stopWatch = new StopWatch();
        lastClick = 0;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if (stopWatch.getTime() - lastClick < Settings.Driver.DOUBLE_CLICK_TIME_BETWEEN_CLICKS) {
            arm.setRequestedState(Arm.State.SPEAKER_HIGH);
        }
        else {
            arm.setRequestedState(Arm.State.SPEAKER_LOW);
        }
        lastClick = stopWatch.getTime();
    }

}
