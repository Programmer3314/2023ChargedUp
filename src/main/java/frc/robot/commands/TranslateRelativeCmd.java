// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

// TODO: Check that this is working during normal (on the floor) driving
// TODO: make this work with a regular pid controller (like TranslateAbsoluteCmd)
/** Add your docs here. */
public class TranslateRelativeCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Pose2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    // private final TrapezoidProfile.Constraints constraints;
    private final PIDController tripPidController;
    private final MMNavigationSubsystem navigationSubsystem;

    public TranslateRelativeCmd(MMSwerveSubsystem swerveSubsystem, Pose2d desiredTranslation, double maxSpeed,
            MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        // constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed / 10);
        tripPidController = new PIDController(4, 0, 0);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = navigationSubsystem.getPose();
        double tripLength = desireTranslation.getTranslation().minus(currentPosition.getTranslation()).getNorm();
        // tripPidController.reset(tripLength);
        targetPosition = currentPosition.getTranslation().plus(desireTranslation.getTranslation());
        SmartDashboard.putString("In LockedIn", "false");
    }

    @Override
    public void execute() {
        Pose2d currentPosition = navigationSubsystem.getPose();
        Translation2d trip = targetPosition.minus(currentPosition.getTranslation());
        double tripLength = targetPosition.getDistance(currentPosition.getTranslation());

        trip = trip.div(trip.getNorm());

        double correction = tripPidController.calculate(tripLength);
        correction *= -1;
        if (correction > maxSpeed) {
            correction = maxSpeed;
        }
        if (correction < -maxSpeed) {
            correction = -maxSpeed;
        }

        swerveSubsystem.drive(trip.getX() * correction, trip.getY() * correction, 0, true,
                navigationSubsystem.getRotation2d());
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
