// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMPIDController;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
// store our rotation for gyro lock in init
// drive slowly forwards
// gyro lock angle
// stop when our speed is near 0
public class DriveToPegCmd extends CommandBase {
    private MMNavigationSubsystem navigationSubsystem;
    private MMSwerveSubsystem swerveSubsystem;
    private double maxSpeed;
    private PIDController tripPidController;
    private MMTurnPIDController turnPidController;
    private Rotation2d gyroLockAngle;
    private boolean pastStartUpFlag;

    public DriveToPegCmd(MMNavigationSubsystem navigationSubsystem, MMSwerveSubsystem swerveSubsystem,
            double maxSpeed) {
        this.navigationSubsystem = navigationSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.maxSpeed = maxSpeed;
        tripPidController = new PIDController(4, 0, 0);
        turnPidController = new MMTurnPIDController();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        gyroLockAngle = navigationSubsystem.getRotation2d();

        turnPidController.initialize(gyroLockAngle);
    }

    @Override
    public void execute() {
        if (!pastStartUpFlag) {
            pastStartUpFlag = swerveSubsystem.getAverageDriveVelocity() > .7;
        }

        double desiredTurn = turnPidController.execute(navigationSubsystem.getHeadingRad());
        double desiredX = Math.cos(gyroLockAngle.getRadians() * maxSpeed);
        double desiredY = Math.sin(gyroLockAngle.getRadians() * maxSpeed);

        swerveSubsystem.drive(desiredX, desiredY, desiredTurn, true,
                navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        boolean droveToPeg = swerveSubsystem.getAverageDriveVelocity() <= .05 && pastStartUpFlag;
        SmartDashboard.putBoolean("Drove To Peg", droveToPeg);
        return droveToPeg;
    }
}
