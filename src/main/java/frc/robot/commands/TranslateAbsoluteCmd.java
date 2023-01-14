// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class TranslateAbsoluteCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Translation2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController tripPidController;

    public TranslateAbsoluteCmd(MMSwerveSubsystem swerveSubsystem, Translation2d desiredTranslation, double maxSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed/10);
        tripPidController = new ProfiledPIDController(4, 0, 0, constraints);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = (desireTranslation);
        // TODO: Try... The problem may be that the initial trip is from 0 to 0
        // double tripLength = desireTranslation.minus(swerveSubsystem.getPose().getTranslation()).getNorm();
        // tripPidController.reset(tripLength);
        SmartDashboard.putString("In LockedIn", "false");
    }

    @Override
    public void execute() {
        Translation2d trip = targetPosition.minus(swerveSubsystem.getPose().getTranslation());
        double tripLength = targetPosition.getDistance(swerveSubsystem.getPose().getTranslation());

        trip = trip.div(trip.getNorm());

        ChassisSpeeds chassisSpeeds;
        double correction = tripPidController.calculate(tripLength);
        correction*=-1;
        if (correction > maxSpeed){
            correction = maxSpeed;
        }
        if (correction <- maxSpeed ){
            correction=-maxSpeed;
        }
           
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                trip.getX() * correction, trip.getY() * correction, 0, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        double tripLength = targetPosition.getDistance(swerveSubsystem.getPose().getTranslation());
        SmartDashboard.putNumber("Trip Length:", tripLength);
        SmartDashboard.putBoolean("Finished Translate", tripLength < .1);
        return tripLength < .1;
    }

}
