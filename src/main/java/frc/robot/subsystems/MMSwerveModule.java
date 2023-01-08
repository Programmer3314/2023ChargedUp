package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

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

        driveMotorController = new WPI_TalonFX(driveMotorCanId);
        driveMotorController.configFactoryDefault();
        driveMotorController.setNeutralMode(NeutralMode.Brake);
        driveMotorController.setInverted(driveMotorReversed);

        turnMotorController = new WPI_TalonFX(turnMotorCanId);
        turnMotorController.configFactoryDefault();
        turnMotorController.setNeutralMode(NeutralMode.Brake);
        turnMotorController.setInverted(turnMotorReversed);
        turnPidController = new PIDController(.65, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();

    }

    public double getDrivePositionMeters() {
        return driveMotorController.getSelectedSensorPosition() * Constants.driveTicksToMeters;
    }

    public double getTurningPositionRadians() {
        // TODO simplify angle, with minimalangle or wpiLib version
        return turnMotorController.getSelectedSensorPosition() * Constants.turnTicksToRadians;
    }

    public double getDriveVelocity() {
        return driveMotorController.getSelectedSensorVelocity() * 10 * Constants.driveTicksToMeters;
    }

    public double getTurningVelocity() {
        return turnMotorController.getSelectedSensorVelocity() * 10 * Constants.turnTicksToRadians;
    }

    public double getAbsoluteEncoderRad() {
        return (magneticCanCoder.getAbsolutePosition() * Math.PI / 180.0) + absoluteEncoderOffset;
    }

    public void resetEncoders() {
        driveMotorController.setSelectedSensorPosition(0);
        turnMotorController.setSelectedSensorPosition(getAbsoluteEncoderRad() / Constants.turnTicksToRadians);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // state = SwerveModuleState.optimize(state, getState().angle);
        driveMotorController.set(ControlMode.PercentOutput,
                state.speedMetersPerSecond / Constants.maxVelocityMetersPerSecond);
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