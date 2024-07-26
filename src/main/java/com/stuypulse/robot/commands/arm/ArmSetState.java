package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmSetState extends InstantCommand{
    
    private final Arm arm;
    private final Arm.State state;

    public ArmSetState(Arm.State state) {
        arm = Arm.getInstance();
        this.state = state;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setRequestedState(state);
    }
}
