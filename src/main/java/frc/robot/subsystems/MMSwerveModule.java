package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class MMSwerveModule {
    private final WPI_TalonFX driveMotorController;
    private final WPI_TalonFX turnMotorController;
    private final WPI_CANCoder magneticCanCoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;
    private final PIDController turnPidController;
    private final int magneticCanCoderId;

    public MMSwerveModule(int driveMotorCanId, int turnMotorCanId, boolean driveMotorReversed,
            boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset,
            boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.magneticCanCoderId = absoluteEncoderId;
        magneticCanCoder = new WPI_CANCoder(magneticCanCoderId);

        magneticCanCoder.configFactoryDefault();
        magneticCanCoder.configMagnetOffset(Math.toDegrees(0));
        magneticCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        driveMotorController = new WPI_TalonFX(driveMotorCanId);
        driveMotorController.configFactoryDefault();

        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveMotorController.getAllConfigs(driveConfigs);
        driveConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveMotorController.configAllSettings(driveConfigs, Constants.Robot.canBusTimeoutMs);
        driveMotorController.setNeutralMode(NeutralMode.Brake);
        driveMotorController.setInverted(driveMotorReversed);

        turnMotorController = new WPI_TalonFX(turnMotorCanId);
        turnMotorController.configFactoryDefault();

        TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
        turnMotorController.getAllConfigs(driveConfigs, Constants.Robot.canBusTimeoutMs);
        turnConfigs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        turnMotorController.configAllSettings(driveConfigs, Constants.Robot.canBusTimeoutMs);
        turnMotorController.setNeutralMode(NeutralMode.Brake);
        turnMotorController.setInverted(turnMotorReversed);

        turnPidController = new PIDController(.65, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        // resetEncoders();

    }

    public double getDrivePositionMeters() {
        return driveMotorController.getSelectedSensorPosition() * Constants.MK4i.L2.driveMetersPerTick;
    }

    public double getTurningPositionRadians() {
        return MathUtil.angleModulus(turnMotorController.getSelectedSensorPosition() * Constants.MK4i.L2.turnRadiansPerTick);
    }

    public double getDriveVelocity() {
        return driveMotorController.getSelectedSensorVelocity() * 10 * Constants.MK4i.L2.driveMetersPerTick;
    }

    public double getTurningVelocity() {
        return turnMotorController.getSelectedSensorVelocity() * 10 * Constants.MK4i.L2.turnRadiansPerTick;
    }

    public double getAbsoluteEncoderRad() {
        return MathUtil.angleModulus((magneticCanCoder.getAbsolutePosition() * Math.PI / 180.0) + absoluteEncoderOffset)
                * (absoluteEncoderReversed ? -1.0 : 1.0);

    }

    public void resetEncoders() {
        driveMotorController.setSelectedSensorPosition(0);
        turnMotorController.setSelectedSensorPosition(getAbsoluteEncoderRad() / Constants.MK4i.L2.turnRadiansPerTick);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.1) {
            stop();
            return;
        }
        setDesiredStateRaw(state);
    };
    public void setDesiredStateRaw(SwerveModuleState state){

    
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotorController.set(ControlMode.PercentOutput,
                state.speedMetersPerSecond / Constants.MK4i.L2.maxVelocityMetersPerSecond);
        turnMotorController.set(turnPidController.calculate(getTurningPositionRadians(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + magneticCanCoderId + "] State", state.toString());

    }

    public void stop() {
        driveMotorController.set(ControlMode.PercentOutput, 0);
        turnMotorController.set(ControlMode.PercentOutput, 0);

    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), new Rotation2d(getTurningPositionRadians()));
    }
}