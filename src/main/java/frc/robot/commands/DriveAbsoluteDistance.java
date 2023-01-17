// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class DriveAbsoluteDistance extends CommandBase{
    private final MMSwerveSubsystem swerveSubsystem;
    private final Translation2d desiredTranslation;
    private final double maxSpeed;
    private SwerveModulePosition[] initialPositions;
    private SwerveModuleState[] desiredState;

    public DriveAbsoluteDistance(MMSwerveSubsystem swerveSubsystem, Translation2d desiredTranslation, double maxSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.desiredTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        addRequirements(swerveSubsystem);
        desiredState = new SwerveModuleState[] {
            new SwerveModuleState(maxSpeed, new Rotation2d(0)),
            new SwerveModuleState(maxSpeed, new Rotation2d(0)),
            new SwerveModuleState(maxSpeed, new Rotation2d(0)),
            new SwerveModuleState(maxSpeed, new Rotation2d(0))
    };
    }
    @Override

    public void initialize(){
        initialPositions = swerveSubsystem.getSwerveModulePositions();
    }

    public void execute(){
        swerveSubsystem.setModuleStatesRaw(desiredState);
    }
    @Override
    public boolean isFinished(){
        SwerveModulePosition[] currentPositions = swerveSubsystem.getSwerveModulePositions();
        SmartDashboard.putNumber("driveAbsoluteDistance:", currentPositions[0].distanceMeters-initialPositions[0].distanceMeters);
        return Math.abs(initialPositions[0].distanceMeters + desiredTranslation.getX() - currentPositions[0].distanceMeters) < .1;
    }
}
