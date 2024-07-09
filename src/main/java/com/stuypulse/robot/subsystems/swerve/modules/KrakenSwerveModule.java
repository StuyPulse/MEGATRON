package com.stuypulse.robot.subsystems.swerve.modules;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.TimedMovingAverage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenSwerveModule extends SwerveModule {

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;

    private final AngleController pivotController;

    private final IFilter targetAcceleration;

    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8); // TODO: Choose n threaqds

    // Control
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
        new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC positionControl =
        new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);

    public KrakenSwerveModule(
        String id, 
        Translation2d location, 
        Rotation2d angleOffset, 
        int driveMotorID, 
        int turnMotorID, 
        int turnEncoderID
    ) {
        super(id, location);

        this.angleOffset = angleOffset;

        driveMotor = new TalonFX(driveMotorID, "*");
        turnMotor = new TalonFX(turnMotorID, "*");
        turnEncoder = new CANcoder(turnEncoderID, "*");

        setDrivePID(Swerve.Drive.kP.doubleValue(), Swerve.Drive.kI.doubleValue(), Swerve.Drive.kD.doubleValue());
        setTurnPID(Swerve.Turn.kP.doubleValue(), Swerve.Turn.kI.doubleValue(), Swerve.Turn.kD.doubleValue());

        // Config Motors
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        driveConfig.MotorOutput.Inverted =
            Swerve.Drive.INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
        turnConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05;
        turnConfig.MotorOutput.Inverted =
            Swerve.Turn.INVERTED.get()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfig.Feedback.SensorToMechanismRatio = Swerve.Drive.L4;
        turnConfig.Feedback.SensorToMechanismRatio = Swerve.Turn.TURN_REDUCTION;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        driveMotor.getConfigurator().apply(driveConfig);
        turnMotor.getConfigurator().apply(turnConfig);

        // TODO: Add current limits?
        // driveConfig.CurrentLimits.StatorCurrentLimit = 65; // 65A stator current limit
        // driveConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Enable stator current limiting

        driveMotor.setPosition(0);

        pivotController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setOutputFilter(x -> -x);

        targetAcceleration = new Derivative()
            .then(new TimedMovingAverage(0.1))
            .then(x -> MathUtil.clamp(x, 0, Settings.Swerve.MAX_MODULE_ACCEL));

        // Optimize bus utilization
        driveMotor.optimizeBusUtilization();
        turnMotor.optimizeBusUtilization();

        // turnMotor is a TalonFX, so we need to configure it as such
        // Motors.disableStatusFrames(turnMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_POSIITION, StatusFrame.ABS_ENCODER_VELOCITY);
        // Motors.Swerve.TURN_CONFIG.configure(turnMotor);
    }

    /****************/
    /*** Getters ***/
    /****************/

    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble() * Encoder.Drive.POSITION_CONVERSION;
    }

    public double getTurnPosition() {
        return turnMotor.getPosition().getValueAsDouble() * Encoder.Turn.POSITION_CONVERSION;
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * Encoder.Drive.POSITION_CONVERSION;
    }

    public double getTurnVelocity() {
        return turnMotor.getVelocity().getValueAsDouble() * Encoder.Turn.POSITION_CONVERSION;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    /****************/
    /*** Setters  ***/
    /****************/

    public void setDrivePID(double kP, double kI, double kD) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveMotor.getConfigurator().apply(driveConfig, 0.01);
    }

    public void setTurnPID(double kP, double kI, double kD) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kI = kI;
        turnConfig.Slot0.kD = kD;
        turnMotor.getConfigurator().apply(turnConfig, 0.01);
    }

    public void setDriveVoltage(double voltage) {
        driveMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void setTurnVoltage(double voltage) {
        turnMotor.setControl(voltageControl.withOutput(voltage));
    }

    public void setCharacterization(double input) {
        driveMotor.setControl(currentControl.withOutput(input));
    }

    public void setDriveVelocity(double velocityRadsPerSec, double feedForward) {
        driveMotor.setControl(
            velocityTorqueCurrentFOC
                .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
                .withFeedForward(feedForward));
    }
    
    public void runTurnPositionSetpoint(double angleRads) {
        turnMotor.setControl(positionControl.withPosition(Units.radiansToRotations(angleRads)));
    }

    public void setDriveBrakeMode(boolean enable) {
        brakeModeExecutor.execute(
            () -> {
                synchronized (driveConfig) {
                    driveConfig.MotorOutput.NeutralMode =
                        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                    driveMotor.getConfigurator().apply(driveConfig, 0.25);
                }
            });
    }

    public void setTurnBrakeMode(boolean enable) {
        brakeModeExecutor.execute(
            () -> {
                synchronized (turnConfig) {
                    turnConfig.MotorOutput.NeutralMode =
                        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                    turnMotor.getConfigurator().apply(turnConfig, 0.25);
                }
            });
    }

    public void stop() {
        driveMotor.setControl(neutralControl);
        turnMotor.setControl(neutralControl);
    }

    private double convertDriveVel(double speedMetersPerSecond) {
        return speedMetersPerSecond / Encoder.Drive.POSITION_CONVERSION;
    }

    @Override
    public void periodic() {
        super.periodic();

        final boolean USE_ACCEL = true;

        double velocity = convertDriveVel(getTargetState().speedMetersPerSecond);
        double acceleration = targetAcceleration.get(velocity);
        boolean useFOC = true;

        if (!USE_ACCEL) {
            acceleration = 0;
        }

        VelocityVoltage driveOutput = new VelocityVoltage(velocity)
            .withAcceleration(acceleration)
            .withEnableFOC(useFOC);

        pivotController.update(Angle.fromRotation2d(getTargetState().angle), Angle.fromRotation2d(getAngle()));

        if (Math.abs(getTargetState().speedMetersPerSecond) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
            driveMotor.setControl(new VelocityVoltage(0));
            turnMotor.setVoltage(0);
        } else {
            driveMotor.setControl(driveOutput);
            turnMotor.setVoltage(pivotController.getOutput());
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Acceleration", acceleration * Encoder.Drive.POSITION_CONVERSION);
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Position", getPosition());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Voltage", pivotController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Current", turnMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle Error", pivotController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Raw Encoder Angle", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
    }
    
}
