package com.stuypulse.robot.subsystems.swerve.controllers;

import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveController {
    private static final SmartNumber controllerDeadband =
        new SmartNumber("Controller/DriveController/Deadband", 0.1);
    private static final SmartNumber maxAngularVelocityScalar =
        new SmartNumber("Controller/DriveController/MaxAngularVelocityScalar", 0.65);


    private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;

    /**
   * Accepts new drive input from joysticks.
   *
   * @param x Desired x velocity scalar, -1..1
   * @param y Desired y velocity scalar, -1..1
   * @param omega Desired omega velocity scalar, -1..1
   */
    public void acceptDriveInput(double x, double y, double omega) {
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
    }

    public ChassisSpeeds update() {
        Translation2d linearVelocity = calcLinearVelocity(controllerX, controllerY);
        double omega = MathUtil.applyDeadband(controllerOmega, controllerDeadband.get());
        omega = Math.copySign(omega * omega, omega);

        final double maxLinearVelocity = Swerve.MAX_LINEAR_VELOCITY;
        final double maxAngularVelocity =
            Swerve.MAX_ANGULAR_VELOCITY * (maxAngularVelocityScalar.get());

        return new ChassisSpeeds(
            linearVelocity.getX() * maxLinearVelocity,
            linearVelocity.getY() * maxLinearVelocity,
            omega * maxAngularVelocity);
    }


    public static Translation2d calcLinearVelocity(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), controllerDeadband.get());
        Rotation2d linearDirection = new Rotation2d(x, y);

        // Square magnitude
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calcaulate new linear velocity
        Translation2d linearVelocity =
            new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
        return linearVelocity;
    }
}
