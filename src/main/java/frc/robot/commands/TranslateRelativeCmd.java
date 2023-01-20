// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class TranslateRelativeCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Pose2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    // private final TrapezoidProfile.Constraints constraints;
    private final PIDController tripPidController;
    private final MMNavigationSubsystem navigationSubsystem;
    private final MMTurnPIDController rotationPidController;

    public TranslateRelativeCmd(MMSwerveSubsystem swerveSubsystem, Pose2d desiredTranslation, double maxSpeed,
            MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        // constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed / 10);
        tripPidController = new PIDController(4, 0, 0);
        rotationPidController = new MMTurnPIDController();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = navigationSubsystem.getPose();
        //double tripLength = desireTranslation.getTranslation().minus(currentPosition.getTranslation()).getNorm();
        // tripPidController.reset(tripLength);
        targetPosition = currentPosition.getTranslation().plus(desireTranslation.getTranslation());
        SmartDashboard.putString("In LockedIn", "false");
        // rotationPidController.setSetpoint(desireTranslation.getRotation().getRadians());
        rotationPidController.initialize(desireTranslation.getRotation());
    }

    @Override
    public void execute() {
        Pose2d currentPosition = navigationSubsystem.getPose();
        Translation2d trip = targetPosition.minus(currentPosition.getTranslation());
        double tripLength = targetPosition.getDistance(currentPosition.getTranslation());
        //double maxRotation = Math.PI / 2.0;

        trip = trip.div(trip.getNorm());
        // rotationPidController.calculate(navigationSubsystem.getRotation2d().getRadians());
        double rotationCorrection = rotationPidController.execute(navigationSubsystem.getRotation2d());
        double correction = tripPidController.calculate(tripLength);
        correction *= -1;
        if (correction > maxSpeed) {
            correction = maxSpeed;
        }
        if (correction < -maxSpeed) {
            correction = -maxSpeed;
        }
        // if (rotationCorrection > maxRotation) {
        //     rotationCorrection = maxRotation;
        // }
        // if (rotationCorrection < -maxRotation) {
        //     rotationCorrection = -maxRotation;
        // }
        SmartDashboard.putNumber("rotationCorrection: ", rotationCorrection);

        swerveSubsystem.drive(trip.getX() * correction, trip.getY() * correction, rotationCorrection, true,
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
        return tripLength < .1 && rotationPidController.isFinished();
    }

}
