// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

// TODO: Try using MMTurnPIDController, but rember that TX is in degrees not radians 
// if this works, do the same for TargetTagCmd

/** Add your docs here. */
public class TargetPegCmd extends CommandBase {

    MMSwerveSubsystem swerveSubsystem;
    double maxRotationSpeed;
    MMNavigationSubsystem navigationSubsystem;
    PIDController turnPidController;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable limelight = inst.getTable("limelight");
    private final double margin;

    public TargetPegCmd(MMSwerveSubsystem swerveSubsystem, double maxRotationSpeed,
            MMNavigationSubsystem navigationSubsystem, double margin) {
        this.swerveSubsystem = swerveSubsystem;
        this.maxRotationSpeed = maxRotationSpeed;
        this.navigationSubsystem = navigationSubsystem;
        this.margin = margin;
        turnPidController = new PIDController(.25, 0, 0);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        navigationSubsystem.changePipeline(1);
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

        swerveSubsystem.drive(0, 0, correction, true, navigationSubsystem.getRotation2d());
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
