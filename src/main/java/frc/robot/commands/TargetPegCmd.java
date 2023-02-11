// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class TargetPegCmd extends CommandBase {

    MMSwerveSubsystem swerveSubsystem;
    double maxRotationSpeed;
    MMNavigationSubsystem navigationSubsystem;
    // MMTurnPIDController turnPidController;
    MMTurnPIDController turnPidController;
    private int cyclesOnTarget;
    Supplier<Boolean> cubeInIntake;
    private boolean isBackCam;

    public TargetPegCmd(MMSwerveSubsystem swerveSubsystem, double maxRotationSpeed,
            MMNavigationSubsystem navigationSubsystem, Supplier<Boolean> cubeInIntake) {
        this.swerveSubsystem = swerveSubsystem;
        this.maxRotationSpeed = maxRotationSpeed;
        this.navigationSubsystem = navigationSubsystem;
        turnPidController = new MMTurnPIDController();
        this.cubeInIntake = cubeInIntake;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // navigationSubsystem.setFrontPipeline(1);
        
        turnPidController.initialize(new Rotation2d());
        cyclesOnTarget = 0;
        isBackCam = cubeInIntake.get();
    }

    @Override
    public void execute() {
        double rawTarget;
        if (isBackCam) {
            rawTarget = navigationSubsystem.getBackTargetX();
            navigationSubsystem.setBackPipeline(1);
        } else {
            rawTarget = navigationSubsystem.getFrontTargetX();
            navigationSubsystem.setFrontPipeline(1);
        }

        Rotation2d targetAngle = new Rotation2d(Math.toRadians(rawTarget));
        double correction = turnPidController.execute(targetAngle.getRadians());
        swerveSubsystem.drive(0, 0, correction, true, navigationSubsystem.getRotation2d());
        if (Math.abs(rawTarget) < 3) {
            cyclesOnTarget++;
        } else {
            cyclesOnTarget = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        if (isBackCam) {
            navigationSubsystem.setBackPipeline(0);
        } else {
            navigationSubsystem.setFrontPipeline(0);
        }

    }

    @Override
    public boolean isFinished() {
        return cyclesOnTarget>=75;
    }
}
