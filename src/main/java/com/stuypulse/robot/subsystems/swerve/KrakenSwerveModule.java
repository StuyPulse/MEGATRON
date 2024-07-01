package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class KrakenSwerveModule extends SwerveModule {
    private final Rotation2d angleOffset;

    // private final TalonFX driveMotor;
    // private final TalonFX turnMotor;
    // private final AnalogInput turnEncoder;

    public KrakenSwerveModule(String id, Translation2d offset, Rotation2d angleOffset) {
        super(id, offset);

        this.angleOffset = angleOffset;        
    }

    @Override
    public Rotation2d getAngle() {
        // return Rotation2d.fromRadians(turnEncoder.getVoltage()
        //                                 / RobotController.getVoltage5V()
        //                                 * 2.0 * Math.PI)
        //                                 .minus(angleOffset);
        throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
    }


    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }


    @Override
    public SwerveModulePosition getModulePosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModulePosition'");
    }   
}
