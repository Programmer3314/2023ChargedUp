// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
public class TranslateAbsoluteCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Pose2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    private final TrapezoidProfile.Constraints constraints;
    private final PIDController tripPidController;
    private final MMnavigationSubsystem navigationSubsystem;
    private final PIDController turnPidController;

    // TODO: Translate has to be updated or replaced to a Pose, doesn't include
    // Rotation here.
    // TODO: Come up with own pid controller, profiled pidController is trash.
    // TODO: In real match, charging station gets in the way, needs command to drive
    // around
    public TranslateAbsoluteCmd(MMSwerveSubsystem swerveSubsystem, Pose2d desiredTranslation, double maxSpeed,
            MMnavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed);
        // tripPidController = new ProfiledPIDController(4, 0, 0, constraints);
        tripPidController = new PIDController(4, 0, 0);
        turnPidController = new PIDController(5, 0, 0);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = desireTranslation.getTranslation();
        // TODO: Try... The problem may be that the initial trip is from 0 to 0
        double tripLength = desireTranslation.getTranslation().minus(navigationSubsystem.getPose().getTranslation())
                .getNorm();
        // tripPidController.reset(tripLength);
        SmartDashboard.putString("In LockedIn", "false");
        turnPidController.setSetpoint(desireTranslation.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Translation2d currentPosition = navigationSubsystem.getPose().getTranslation();
        Translation2d trip = targetPosition.minus(currentPosition);
        double tripLength = targetPosition.getDistance(currentPosition);
        double desiredTurn = turnPidController.calculate(navigationSubsystem.getHeadingRad());

        trip = trip.div(trip.getNorm());

        ChassisSpeeds chassisSpeeds;
        double correction = tripPidController.calculate(tripLength);
        correction *= -1;
        if (correction > maxSpeed) {
            correction = maxSpeed;
        }
        if (correction < -maxSpeed) {
            correction = -maxSpeed;
        }

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                trip.getX() * correction, trip.getY() * correction, desiredTurn, navigationSubsystem.getRotation2d());
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
        return tripLength < .05;
    }

}
