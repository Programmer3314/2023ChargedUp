// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DriveAbsoluteDistanceCmd extends CommandBase {
    private final Translation2d desiredTranslation;
    private final double maxSpeed;
    private SwerveModulePosition[] initialPositions;
    private PIDController turnPidController;
    private Rotation2d initialYaw;
    private RobotContainer rc;

    // TODO: Try using MMTurnPIDController

    // TODO: low priority implement full vectored driving
    // (forward/backward,left/right (no
    // rotation))
    public DriveAbsoluteDistanceCmd(RobotContainer rc, Translation2d desiredTranslation, double maxSpeed) {
        this.rc = rc;
        this.desiredTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        turnPidController = new PIDController(-5, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(rc.swerveSubsystem);
    }

    @Override

    public void initialize() {
        initialPositions = rc.swerveSubsystem.getSwerveModulePositions();
        initialYaw = rc.navigationSubsystem.getYaw();
        turnPidController.setSetpoint(initialYaw.getRadians());
        SmartDashboard.putString("Robot Starting Pos: ", rc.navigationSubsystem.getPose().getTranslation().toString());
    }

    public void execute() {
        rc.swerveSubsystem.drive(maxSpeed, 0,
                turnPidController.calculate(rc.navigationSubsystem.getYaw().getRadians()),
                false, new Rotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        SwerveModulePosition[] currentPositions = rc.swerveSubsystem.getSwerveModulePositions();
        SmartDashboard.putNumber("driveAbsoluteDistance:",
                currentPositions[3].distanceMeters - initialPositions[3].distanceMeters);
        return Math.abs(initialPositions[3].distanceMeters - currentPositions[3].distanceMeters) > Math
                .abs(desiredTranslation.getX());
    }
}
