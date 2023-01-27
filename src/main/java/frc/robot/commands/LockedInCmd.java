// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class LockedInCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    SwerveModuleState[] desiredState = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4.0)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4.0)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4.0)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4.0))
    };

    public LockedInCmd(MMSwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SmartDashboard.putString("In LockedIn", "true");
        swerveSubsystem.setModuleStatesRaw(desiredState,true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
