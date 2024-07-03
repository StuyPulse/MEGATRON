package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSim extends Intake {

    private double funnelMotorHigh;
    private double funnelMotorLow;
    private double intakeMotor;

    public IntakeSim() {
        funnelMotorHigh = 0;
        funnelMotorLow = 0;
        intakeMotor = 0;
    }

    @Override
    public void acquire() {
        intakeMotor = +Settings.Intake.ACQUIRE_SPEED;
    }

    @Override
    public void deacquire(){
        intakeMotor = -Settings.Intake.DEACQUIRE_SPEED;
    }

    @Override
    public void stopIntake() {
        intakeMotor = 0;
    }

    @Override
    public void funnel() {
        funnelMotorHigh = +Settings.Intake.TOP_FUNNEL_ACQUIRE;
        funnelMotorLow = +Settings.Intake.BOTTOM_FUNNEL_ACQUIRE;
    }

    @Override
    public void defunnel() {
        funnelMotorHigh = -Settings.Intake.TOP_FUNNEL_DEACQUIRE;
        funnelMotorLow = -Settings.Intake.BOTTOM_FUNNEL_DEACQUIRE;
    }

    @Override
    public void stopFunnel() {
        funnelMotorHigh = 0;
        funnelMotorLow = 0;
    }

    @Override
    public double getIntakeRollerSpeed() {
        return intakeMotor;
    }

    @Override
    public double getTopFunnelRollerSpeed() {
        return funnelMotorHigh;
    }

    @Override
    public double getLowFunnelRollerSpeed() {
        return funnelMotorLow;
    }

@Override
public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor);
    SmartDashboard.putNumber("Intake/Funnel/Top-Funnel Speed", funnelMotorHigh);
    SmartDashboard.putNumber("Intake/Funnel/Low-Funnel Speed", funnelMotorLow);
}

}
