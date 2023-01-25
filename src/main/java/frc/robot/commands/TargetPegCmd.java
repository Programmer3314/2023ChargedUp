// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMTurnPIDController;


/** Add your docs here. */
public class TargetPegCmd extends CommandBase {

    MMSwerveSubsystem swerveSubsystem;
    double maxRotationSpeed;
    MMNavigationSubsystem navigationSubsystem;
    //MMTurnPIDController turnPidController;
    MMTurnPIDController turnPidController;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable limelight = inst.getTable(Constants.Limelight.fLimelight);

    public TargetPegCmd(MMSwerveSubsystem swerveSubsystem, double maxRotationSpeed,
            MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.maxRotationSpeed = maxRotationSpeed;
        this.navigationSubsystem = navigationSubsystem;
        turnPidController = new MMTurnPIDController();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        navigationSubsystem.setPipeline(1);
        turnPidController.initialize(new Rotation2d());
    }

    @Override
    public void execute() {
        Rotation2d targetAngle = new Rotation2d(Math.toRadians(limelight.getEntry("tx").getDouble(0)));
        double correction = turnPidController.execute(targetAngle.getRadians());
        swerveSubsystem.drive(0, 0, correction, true, navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        navigationSubsystem.setPipeline(0);
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(limelight.getEntry("tx").getDouble(0)) < margin;
        return false;
    }
}
