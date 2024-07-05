package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSim extends Intake {

    private double funnelMotorLeft;
    private double funnelMotorRight;
    private double intakeMotor;

    public IntakeSim() {
        funnelMotorLeft = 0;
        funnelMotorRight = 0;
        intakeMotor = 0;
    }

    @Override
    public void acquire() {
        intakeMotor = +Settings.Intake.INTAKE_ACQUIRE_SPEED;
        funnelMotorLeft = +Settings.Intake.FUNNEL_ACQUIRE;
        funnelMotorRight = +Settings.Intake.FUNNEL_ACQUIRE;
    }

    @Override
    public void deacquire(){
        intakeMotor = -Settings.Intake.INTAKE_DEACQUIRE_SPEED;
        funnelMotorLeft = -Settings.Intake.FUNNEL_DEACQUIRE;
        funnelMotorRight = -Settings.Intake.FUNNEL_DEACQUIRE;
    }

    @Override
    public void stop() {
        intakeMotor = 0;
        funnelMotorLeft = 0;
        funnelMotorRight = 0;
    }

    @Override
    public boolean hasNote() {
        return false;
    }
    
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor);
        SmartDashboard.putNumber("Intake/Funnel/Left Funnel Speed", funnelMotorLeft);
        SmartDashboard.putNumber("Intake/Funnel/Right Funnel Speed", funnelMotorRight);
    }

}
