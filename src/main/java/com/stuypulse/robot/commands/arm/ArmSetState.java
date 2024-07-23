package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSetState extends Command{
    
    private final Arm arm;
    private final Arm.State state;
    private final StopWatch stopWatch;

    public ArmSetState(Arm.State state) {
        arm = Arm.getInstance();
        this.state = state;
        this.stopWatch = new StopWatch();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRequestedState(state);
        stopWatch.reset();
    }

    @Override
    public void execute() {
        arm.setRequestedState(state);
        if (stopWatch.getTime() > Settings.Arm.HOLD_TO_OVERRIDE_TIME) {
            arm.setOverriding(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setOverriding(false);
    }
}
