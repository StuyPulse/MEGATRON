package com.stuypulse.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;

public class IntakeImpl extends Intake {

    private CANSparkMax funnelMotorHigh;
    private CANSparkMax funnelMotorLow;
    private CANSparkMax intakeMotor;

    public IntakeImpl() {
        funnelMotorHigh = new CANSparkMax(Ports.Intake.FUNNEL_HIGH, MotorType.kBrushless);
        funnelMotorLow = new CANSparkMax(Ports.Intake.FUNNEL_LOW, MotorType.kBrushless);        
        intakeMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, MotorType.kBrushless);

        Motors.Intake.TOP_FUNNEL_MOTOR_CONFIG.configure(funnelMotorHigh);
        Motors.Intake.LOW_FUNNEL_MOTOR_CONFIG.configure(funnelMotorLow);
        Motors.Intake.INTAKE_MOTOR_CONFIG.configure(intakeMotor);
    }

@Override
public void acquire() {
    intakeMotor.set(+Settings.Intake.ACQUIRE_SPEED);
}

@Override
public void deacquire() {
    intakeMotor.set(-Settings.Intake.DEACQUIRE_SPEED);
}

@Override
public void stopIntake() {
    intakeMotor.stopMotor();
}

@Override
public void funnel() {
    funnelMotorHigh.set(+Settings.Intake.TOP_FUNNEL_ACQUIRE);
    funnelMotorLow.set(+Settings.Intake.BOTTOM_FUNNEL_ACQUIRE);
}

@Override
public void defunnel() {
    funnelMotorHigh.set(-Settings.Intake.TOP_FUNNEL_DEACQUIRE);
    funnelMotorLow.set(-Settings.Intake.BOTTOM_FUNNEL_DEACQUIRE);
}

@Override
public void stopFunnel() {
    funnelMotorHigh.stopMotor();
    funnelMotorLow.stopMotor();
}

@Override
public double getIntakeRollerSpeed() {
    return intakeMotor.get();
}

@Override
public double getTopFunnelRollerSpeed() {
    return funnelMotorHigh.get();
}

@Override
public double getLowFunnelRollerSpeed() {
    return funnelMotorLow.get();
}


@Override
public void periodic() {
    super.periodic();
}

}