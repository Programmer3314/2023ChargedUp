// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class DriveToRampCmd extends CommandBase {
    private double maxSpeed;
    private double maxAngle;
    private double robotPitch;
    private RobotContainer rc;

    public DriveToRampCmd(RobotContainer rc,
            double maxSpeed) {
        this.maxSpeed = maxSpeed;
        addRequirements(rc.swerveSubsystem);
        robotPitch = rc.navigationSubsystem.getPitch();
    }

    @Override
    public void initialize() {
        maxAngle = rc.navigationSubsystem.getPitch();
    }

    @Override
    public void execute() {
        robotPitch = rc.navigationSubsystem.getPitch();
        if (maxAngle < robotPitch) {
            maxAngle = robotPitch;
        }
        SmartDashboard.putNumber("maxAngle", maxAngle);
        SmartDashboard.putNumber("currentPitch", robotPitch);
        rc.swerveSubsystem.drive(maxSpeed, 0, 0, true, rc.navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        boolean finishedCondition = robotPitch < (maxAngle - 3);
        SmartDashboard.putBoolean("Finished Condition:", finishedCondition);
        return finishedCondition;
    }
}
