// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    public DriveToRampCmd(MMSwerveSubsystem swerveSubsystem, MMNavigationSubsystem navigationSubsystem,
            double maxSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.maxSpeed = maxSpeed;
    }

    @Override
    public void initialize() {
        addRequirements(swerveSubsystem);
        angleDecreaseFlag = false;
        maxAngle = -navigationSubsystem.getRoll();
    }

    @Override
    public void execute() {
      //  ChassisSpeeds chassisSpeeds;
        SmartDashboard.putBoolean("angleFlag", angleDecreaseFlag);
        if(maxAngle<-navigationSubsystem.getRoll()){
            maxAngle = -navigationSubsystem.getRoll();
        }
        SmartDashboard.putNumber("maxAngle", maxAngle);
        SmartDashboard.putNumber("currentRoll", -navigationSubsystem.getRoll());
        // TODO: X This should use swerveSubsystem.Drive method
        // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(maxSpeed, 0, 0, navigationSubsystem.getRotation2d());
        // SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);
        // swerveSubsystem.setModuleStates(moduleStates);
        //
swerveSubsystem.drive(maxSpeed, 0, 0, true, navigationSubsystem.getRotation2d());
        // if (navigationSubsystem.getRoll() < -16) {
        //     angleDecreaseFlag = true;
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
        return -navigationSubsystem.getRoll()<(maxAngle-1);
    }
}
