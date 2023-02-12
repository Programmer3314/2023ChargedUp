// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class TranslateAbsoluteCmd extends CommandBase {
    private final Supplier<Pose2d> desiredPosition;
    private final double maxSpeed;
    private Translation2d targetPosition;
    // private final TrapezoidProfile.Constraints constraints;
    private final PIDController tripPidController;
    private final MMTurnPIDController turnPidController;
    private final RobotContainer rc;

    public TranslateAbsoluteCmd(RobotContainer rc, Supplier<Pose2d> desiredTranslation, double maxSpeed) {
        this.desiredPosition = desiredTranslation;
        this.maxSpeed = maxSpeed;
        tripPidController = new PIDController(4, 0, 0);
        turnPidController = new MMTurnPIDController();
        this.rc=rc;

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = desiredPosition.get().getTranslation();
        SmartDashboard.putString("Target Position", targetPosition.toString());
        SmartDashboard.putString("In LockedIn", "false");
        turnPidController.initialize(desiredPosition.get().getRotation().getRadians());
    }

    @Override
    public void execute() {

        Translation2d currentPosition = rc.navigationSubsystem.getPose().getTranslation();
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

        double desiredTurn = turnPidController.execute(rc.navigationSubsystem.getHeadingRad());

        rc.swerveSubsystem.drive(trip.getX() * correction, trip.getY() * correction, desiredTurn, true,
                rc.navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {

        double tripLength = targetPosition.getDistance(rc.navigationSubsystem.getPose().getTranslation());
        boolean finishedTranslate = tripLength < .05;
        SmartDashboard.putNumber("Trip Length:", tripLength);
        SmartDashboard.putBoolean("Finished Translate", finishedTranslate);
        return finishedTranslate && turnPidController.isFinished();
    }

}
