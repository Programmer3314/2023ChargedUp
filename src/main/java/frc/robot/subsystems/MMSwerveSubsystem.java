// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class MMSwerveSubsystem extends SubsystemBase {
    private final MMSwerveModule[] modules = new MMSwerveModule[] {
            new MMSwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                    Constants.driveIsInverted, Constants.steerIsInverted, Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                    Constants.FRONT_LEFT_MODULE_STEER_OFFSET, Constants.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                    Constants.driveIsInverted, Constants.steerIsInverted, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                    Constants.FRONT_RIGHT_MODULE_STEER_OFFSET, Constants.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                    Constants.driveIsInverted, Constants.steerIsInverted, Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                    Constants.BACK_RIGHT_MODULE_STEER_OFFSET, Constants.absoluteEncoderIsInverted),
            new MMSwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                    Constants.driveIsInverted, Constants.steerIsInverted, Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                    Constants.BACK_LEFT_MODULE_STEER_OFFSET, Constants.absoluteEncoderIsInverted)

    };

    // private final AHRS navx= new AHRS(SPI.Port.KMXP);
    public MMSwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(3000);
                zeroHeading();
                resetEncoders();
            } catch (Exception e) {

            }
        });
    }

    public void zeroHeading() {
        // navx.reset();
    }

    public double getHeading() {
        // return Math.IEEEremainder(navx.getAngle(), 360);
        return 0;

    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxVelocityMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void resetEncoders() {
        for (MMSwerveModule m : modules) {
            m.resetEncoders();
        }
    }

}
