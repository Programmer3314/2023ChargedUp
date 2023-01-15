// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.subsystems.MMnavigationSubsystem;

/** Add your docs here. */
public class TargetTagCmd extends CommandBase{
    MMSwerveSubsystem swerveSubsystem;
    double maxRotationSpeed;
    MMnavigationSubsystem navigationSubsystem;
    PIDController turnPidController;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable limelight = inst.getTable("limelight");
    private final double margin;

    public TargetTagCmd(MMSwerveSubsystem swerveSubsystem, double maxRotationSpeed,
            MMnavigationSubsystem navigationSubsystem, double margin) {
        this.swerveSubsystem = swerveSubsystem;
        this.maxRotationSpeed = maxRotationSpeed;
        this.navigationSubsystem = navigationSubsystem;
        this.margin = margin;
        addRequirements(swerveSubsystem);
        turnPidController = new PIDController(.175, 0, 0);
    }

    @Override
    public void initialize() {
        navigationSubsystem.changePipeline(0);
    }

    @Override
    public void execute() {
        Rotation2d targetAngle = new Rotation2d(limelight.getEntry("tx").getDouble(0));
        double correction = turnPidController.calculate(targetAngle.getRadians());
        if (correction > maxRotationSpeed) {
            correction = maxRotationSpeed;
        }
        if (correction < -maxRotationSpeed) {
            correction = -maxRotationSpeed;
        }
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, correction, navigationSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(limelight.getEntry("tx").getDouble(0)) < margin;
        return false;
    }
}
