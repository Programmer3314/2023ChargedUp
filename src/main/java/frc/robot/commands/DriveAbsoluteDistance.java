// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class DriveAbsoluteDistance extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Translation2d desiredTranslation;
    private final double maxSpeed;
    private SwerveModulePosition[] initialPositions;
    private SwerveModuleState[] desiredState;
    private PIDController turnPidController;
    private Rotation2d initialYaw;
    private MMNavigationSubsystem navigationSubsystem;

    // TODO: low priority implement full vectored driving
    // (forward/backward,left/right (no
    // rotation))
    public DriveAbsoluteDistance(MMSwerveSubsystem swerveSubsystem, Translation2d desiredTranslation, double maxSpeed,
            MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desiredTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        addRequirements(swerveSubsystem);
        desiredState = new SwerveModuleState[] {
                new SwerveModuleState(maxSpeed, new Rotation2d(0)),
                new SwerveModuleState(maxSpeed, new Rotation2d(0)),
                new SwerveModuleState(maxSpeed, new Rotation2d(0)),
                new SwerveModuleState(maxSpeed, new Rotation2d(0))
        };
        turnPidController = new PIDController(-5, 0, 0);
    }

    @Override

    public void initialize() {
        initialPositions = swerveSubsystem.getSwerveModulePositions();
        initialYaw = navigationSubsystem.getYaw();
        turnPidController.setSetpoint(initialYaw.getRadians());
    }

    public void execute() {
        // swerveSubsystem.setModuleStates(desiredState);
        swerveSubsystem.drive(maxSpeed, 0, turnPidController.calculate(navigationSubsystem.getYaw().getRadians()),
                false, new Rotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        SwerveModulePosition[] currentPositions = swerveSubsystem.getSwerveModulePositions();
        SmartDashboard.putNumber("driveAbsoluteDistance:",
                currentPositions[0].distanceMeters - initialPositions[0].distanceMeters);

        // TODO: clean this up & and fix references
        // like: abs(change in position)>=abs(length of trip)
        return Math.abs(initialPositions[0].distanceMeters - currentPositions[0].distanceMeters) > Math
                .abs(desiredTranslation.getX());
    }
}
