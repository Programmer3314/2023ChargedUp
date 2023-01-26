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
    private final PIDController tripPidController;
    private final MMNavigationSubsystem navigationSubsystem;
    private final MMTurnPIDController rotationPidController;

    public TranslateRelativeCmd(MMSwerveSubsystem swerveSubsystem, Pose2d desiredTranslation, double maxSpeed,
            MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        tripPidController = new PIDController(4, 0, 0);
        rotationPidController = new MMTurnPIDController();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = navigationSubsystem.getPose();
        targetPosition = currentPosition.getTranslation().plus(desireTranslation.getTranslation());
        SmartDashboard.putString("In LockedIn", "false");
        rotationPidController.initialize(desireTranslation.getRotation());
    }

    @Override
    public void execute() {
        Pose2d currentPosition = navigationSubsystem.getPose();
        Translation2d trip = targetPosition.minus(currentPosition.getTranslation());
        double tripLength = targetPosition.getDistance(currentPosition.getTranslation());

        trip = trip.div(trip.getNorm());
        double rotationCorrection = rotationPidController.execute(navigationSubsystem.getRotation2d());
        double correction = tripPidController.calculate(tripLength);
        correction *= -1;
        if (correction > maxSpeed) {
            correction = maxSpeed;
        }
        if (correction < -maxSpeed) {
            correction = -maxSpeed;
        }
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
        boolean finishedTranslate = tripLength < .1;
        SmartDashboard.putNumber("Trip Length:", tripLength);
        SmartDashboard.putBoolean("Finished Translate", finishedTranslate);
        return finishedTranslate && rotationPidController.isFinished();
    }

}
