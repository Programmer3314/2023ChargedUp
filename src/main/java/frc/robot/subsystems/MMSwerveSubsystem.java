// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class MMSwerveSubsystem extends SubsystemBase {
    private final MMSwerveModule[] modules = new MMSwerveModule[] {
            new MMSwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                    Constants.FRONT_LEFT_MODULE_STEER_OFFSET, Constants.MK4i.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                    Constants.FRONT_RIGHT_MODULE_STEER_OFFSET, Constants.MK4i.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                    Constants.BACK_RIGHT_MODULE_STEER_OFFSET, Constants.MK4i.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                    Constants.MK4i.driveIsInverted, Constants.MK4i.steerIsInverted,
                    Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                    Constants.BACK_LEFT_MODULE_STEER_OFFSET, Constants.MK4i.absoluteEncoderIsInverted)

    };

    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.Chassis.kinematics, new Rotation2d(),
            getSwerveModulePositions());

    public MMSwerveSubsystem() {
        SmartDashboard.putString("Reset Running: ", "No");
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                resetEncoders();
                SmartDashboard.putString("Reset Running: ", "Yes");
            } catch (Exception e) {
                SmartDashboard.putString("Reset Running: ", "Error");
            }
        }).start();
        // resetEncoders();
        // zeroHeading();
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeadingRad() {
        return -MathUtil.angleModulus(navx.getAngle() * Math.PI / 180.0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getHeadingRad());

    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePositions());
        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(getHeadingRad()));
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Absolute Encoder Rotation " + i,
                    Math.toDegrees(modules[i].getAbsoluteEncoderRad()));
            SmartDashboard.putNumber("Motor  Rotation" + i, Math.toDegrees(modules[i].getTurningPositionRadians()));
        }
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    }

    public void stopModules() {
        for (MMSwerveModule m : modules) {
            m.stop();
        }

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MK4i.L2.maxVelocityMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void setModuleStatesRaw(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MK4i.L2.maxVelocityMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredStateRaw(desiredStates[i]);
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

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }
}
