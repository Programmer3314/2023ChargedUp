// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class translateCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Translation2d desireTranslation;
    private final double maxSpeed;

    public translateCmd(MMSwerveSubsystem swerveSubsystem, Translation2d desiredTranslation, double maxSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        // calculate the distance from the initial position to the target position
        // needs odometry
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        //ever execution, recalculated the positions needed to get from current position to the target
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
