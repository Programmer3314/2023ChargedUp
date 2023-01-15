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
import frc.robot.subsystems.MMnavigationSubsystem;

/** Add your docs here. */
public class TranslateRelativeCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Translation2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController tripPidController;
    private final MMnavigationSubsystem navigationSubsystem;

    public TranslateRelativeCmd(MMSwerveSubsystem swerveSubsystem, Translation2d desiredTranslation, double maxSpeed,
            MMnavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed / 10);
        tripPidController = new ProfiledPIDController(4, 0, 0, constraints);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Translation2d currentPosition = navigationSubsystem.getPose().getTranslation();
        double tripLength = desireTranslation.minus(currentPosition).getNorm();
        tripPidController.reset(tripLength);
        targetPosition = currentPosition.plus(desireTranslation);
        // TODO: mimic fix (if it works) from TranslateAbsoluteCmd
        SmartDashboard.putString("In LockedIn", "false");
    }

    @Override
    public void execute() {
        Translation2d currentPosition = navigationSubsystem.getPose().getTranslation();
        Translation2d trip = targetPosition.minus(currentPosition);
        double tripLength = targetPosition.getDistance(currentPosition);

        trip = trip.div(trip.getNorm());

        ChassisSpeeds chassisSpeeds;
        double correction = tripPidController.calculate(tripLength);
        correction *= -1;
        if (correction > maxSpeed){
        correction = maxSpeed;
        }
        if (correction <- maxSpeed ){
        correction=-maxSpeed;
        }

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                trip.getX() * correction, trip.getY() * correction, 0, navigationSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {

        double tripLength = targetPosition.getDistance(navigationSubsystem.getPose().getTranslation());
        SmartDashboard.putNumber("Trip Length:", tripLength);
        SmartDashboard.putBoolean("Finished Translate", tripLength < .1);
        return tripLength < .1;
    }

}
