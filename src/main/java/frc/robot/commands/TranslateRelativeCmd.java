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
public class TranslateRelativeCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Translation2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController tripPidController;

    public TranslateRelativeCmd(MMSwerveSubsystem swerveSubsystem, Translation2d desiredTranslation, double maxSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed);
        // TODO: try increasing kP and create constant(s) to be used by TranslateAbsoluteCmd
        tripPidController = new ProfiledPIDController(1, 0, 0, constraints);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = swerveSubsystem.getPose().getTranslation().plus(desireTranslation);
        SmartDashboard.putString("In LockedIn", "false");
    }

    @Override
    public void execute() {
        Translation2d trip = targetPosition.minus(swerveSubsystem.getPose().getTranslation());
        double tripLength = targetPosition.getDistance(swerveSubsystem.getPose().getTranslation());

        // TODO: replace getDistance(...) with getNorm()
        trip = trip.div(trip.getDistance(new Translation2d()));

        ChassisSpeeds chassisSpeeds;
        double correction = tripPidController.calculate(tripLength);
        correction*=-1;

        // TODO: add minimum/nominal value for small corrections
    
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                trip.getX() * correction, trip.getY() * correction, 0, swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = Constants.Chassis.kinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
        // every execution, recalculated the positions needed to get from current
        // position to the target

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
