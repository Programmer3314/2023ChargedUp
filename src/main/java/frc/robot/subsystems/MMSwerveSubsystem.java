// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class MMSwerveSubsystem extends SubsystemBase {
    private final MMSwerveModule[] modules = new MMSwerveModule[] {
            new MMSwerveModule(Constants.Chassis.frontLeftModule.driveMotor,
                    Constants.Chassis.frontLeftModule.steerMotor,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.Chassis.frontLeftModule.absoluteEncoderId,
                    Constants.Chassis.frontLeftModule.absoluteEncoderOffset,
                    Constants.MK4i.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.Chassis.frontRightModule.driveMotor,
                    Constants.Chassis.frontRightModule.steerMotor,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.Chassis.frontRightModule.absoluteEncoderId,
                    Constants.Chassis.frontRightModule.absoluteEncoderOffset,
                    Constants.MK4i.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.Chassis.backRightModule.driveMotor,
                    Constants.Chassis.backRightModule.steerMotor,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.Chassis.backRightModule.absoluteEncoderId,
                    Constants.Chassis.backRightModule.absoluteEncoderOffset,
                    Constants.MK4i.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.Chassis.backLeftModule.driveMotor,
                    Constants.Chassis.backLeftModule.steerMotor,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.Chassis.backLeftModule.absoluteEncoderId,
                    Constants.Chassis.backLeftModule.absoluteEncoderOffset,
                    Constants.MK4i.absoluteEncoderIsInverted)
    };

    public MMSwerveSubsystem() {
        SmartDashboard.putString("Reset Running: ", "No");
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetEncoders();
                SmartDashboard.putString("Reset Running: ", "Yes");
            } catch (Exception e) {
                SmartDashboard.putString("Reset Running: ", "Error");
            }
        }).start();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Absolute Encoder Rotation " + i,
                    Math.toDegrees(modules[i].getAbsoluteEncoderRad()));
            SmartDashboard.putNumber("Motor  Rotation" + i, Math.toDegrees(modules[i].getTurningPositionRadians()));
        }
    }

    public void stopModules() {
        for (MMSwerveModule m : modules) {
            m.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MK4i.L1.maxVelocityMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void setModuleStatesRaw(SwerveModuleState[] desiredStates, boolean usePercentOutput) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MK4i.L1.maxVelocityMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredStateRaw(desiredStates[i], usePercentOutput);
        }
    }

    public void resetEncoders() {
        for (MMSwerveModule m : modules) {
            m.resetEncoders();
        }
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                modules[0].getSwerveModulePosition(),
                modules[1].getSwerveModulePosition(),
                modules[2].getSwerveModulePosition(),
                modules[3].getSwerveModulePosition()
        };
    }

    public double getAverageDriveVelocity() {
        double avgVelocity = 0;
        for (int i = 0; i < 4; i++) {
            avgVelocity += Math.abs(modules[i].getDriveVelocity());
        }
        return avgVelocity / 4.0;
    }

    public void drive(double xMetersPerSec, double yMetersPerSec, double rRadPerSec, boolean isFieldCentric,
            Rotation2d robotAngle) {
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSec, yMetersPerSec, rRadPerSec, isFieldCentric ? robotAngle : new Rotation2d());
        SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void humanDrive(double xMetersPerSec, double yMetersPerSec, double rRadPerSec, boolean isFieldCentric,
            Rotation2d robotAngle, Supplier<Boolean> isRedAlliance) {
        Rotation2d reversedRobotAngle = isRedAlliance.get() ? new Rotation2d(robotAngle.getRadians() + Math.PI) : robotAngle;
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSec, yMetersPerSec, rRadPerSec, isFieldCentric ? reversedRobotAngle : new Rotation2d());
        SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }
}