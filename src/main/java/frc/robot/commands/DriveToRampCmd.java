// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

// TODO: Recode this without any specific angles. 
/** Add your docs here. */
public class DriveToRampCmd extends CommandBase {
    private MMSwerveSubsystem swerveSubsystem;
    private MMNavigationSubsystem navigationSubsystem;
    private double maxSpeed;
    private boolean angleDecreaseFlag = false;
    private double maxAngle;
    private double robotRoll;

    public DriveToRampCmd(MMSwerveSubsystem swerveSubsystem, MMNavigationSubsystem navigationSubsystem,
            double maxSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.maxSpeed = maxSpeed;
        addRequirements(swerveSubsystem);
        robotRoll=-navigationSubsystem.getRoll();
    }

    @Override
    public void initialize() {
        angleDecreaseFlag = false;
        maxAngle = -navigationSubsystem.getRoll();
    }

    @Override
    public void execute() {
        robotRoll=-navigationSubsystem.getRoll();
        // SmartDashboard.putBoolean("angleFlag", angleDecreaseFlag);
        // TODO: get roll once and store in class variable
        if (maxAngle < robotRoll) {
            maxAngle = robotRoll;
        }
        SmartDashboard.putNumber("maxAngle", maxAngle);
        SmartDashboard.putNumber("currentRoll", robotRoll);
        swerveSubsystem.drive(maxSpeed, 0, 0, true, navigationSubsystem.getRotation2d());
        // if (navigationSubsystem.getRoll() < -16) {
        // angleDecreaseFlag = true;
        // }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // double currentPitch = navigationSubsystem.getRoll();
        // return angleDecreaseFlag && currentPitch > -14;
        SmartDashboard.putBoolean("Finished COndition:", robotRoll< (maxAngle - 1));
        return robotRoll< (maxAngle - 3);
    }
}
