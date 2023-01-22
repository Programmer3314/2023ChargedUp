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
public class TranslateAbsoluteCmd extends CommandBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final Pose2d desireTranslation;
    private final double maxSpeed;
    private Translation2d targetPosition;
    //private final TrapezoidProfile.Constraints constraints;
    private final PIDController tripPidController;
    private final MMNavigationSubsystem navigationSubsystem;
    private final MMTurnPIDController turnPidController;

    public TranslateAbsoluteCmd(MMSwerveSubsystem swerveSubsystem, Pose2d desiredTranslation, double maxSpeed,
            MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.desireTranslation = desiredTranslation;
        this.maxSpeed = maxSpeed;
        this.navigationSubsystem = navigationSubsystem;
        //constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed);
        tripPidController = new PIDController(4, 0, 0);
        turnPidController=new MMTurnPIDController();
        // turnPidController = new PIDController(5, 0, 0);
        // turnPidController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = desireTranslation.getTranslation();
        SmartDashboard.putString("In LockedIn", "false");
        turnPidController.initialize(desireTranslation.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Translation2d currentPosition = navigationSubsystem.getPose().getTranslation();
        Translation2d trip = targetPosition.minus(currentPosition);
        double tripLength = trip.getNorm();

        trip = trip.div(tripLength);

        double correction = tripPidController.calculate(tripLength);
        correction *= -1;
        if (correction > maxSpeed) {
            correction = maxSpeed;
        }
        if (correction < -maxSpeed) {
            correction = -maxSpeed;
        }

        double desiredTurn = turnPidController.execute(navigationSubsystem.getHeadingRad());

        swerveSubsystem.drive(trip.getX() * correction, trip.getY() * correction, desiredTurn, true,
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
        return tripLength < .05 &&turnPidController.isFinished();
    }

}
