// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class TargetTagCmd extends CommandBase {
    MMSwerveSubsystem swerveSubsystem;
    double maxRotationSpeed;
    MMNavigationSubsystem navigationSubsystem;
    MMTurnPIDController turnPidController;
    Supplier<Boolean> cubeInIntake;
    int cyclesOnTarget;

    public TargetTagCmd(MMSwerveSubsystem swerveSubsystem, double maxRotationSpeed,
            MMNavigationSubsystem navigationSubsystem, Supplier<Boolean> cubeInIntake) {
        this.swerveSubsystem = swerveSubsystem;
        this.maxRotationSpeed = maxRotationSpeed;
        this.navigationSubsystem = navigationSubsystem;
        turnPidController = new MMTurnPIDController();
        this.cubeInIntake=cubeInIntake;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        navigationSubsystem.setFrontPipeline(0);
        turnPidController.initialize(new Rotation2d());
        cyclesOnTarget=0;
    }

    @Override
    public void execute() {
        double rawTarget;
        if (cubeInIntake.get()) {
            rawTarget = navigationSubsystem.getBackTargetX();
        } else {
            rawTarget = navigationSubsystem.getFrontTargetX();
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
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(limelight.getEntry("tx").getDouble(0)) < margin;
        return cyclesOnTarget>=75;
    }
}
