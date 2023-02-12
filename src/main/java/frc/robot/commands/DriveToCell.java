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
import frc.robot.utility.MMField;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class DriveToCell extends CommandBase {
    private Supplier<Integer> desiredCell;
    private RobotContainer rc;
    private double maxSpeed;
    private Translation2d targetPosition;
    private final PIDController tripPidController;
    private final MMTurnPIDController turnPidController;
    private Pose2d targetPose2d;
    private Supplier<Boolean> isRedAlliance;

    public DriveToCell(RobotContainer rc, Supplier<Integer> desiredCell, Supplier<Boolean> isRedAlliance,
            double maxSpeed) {
        this.desiredCell = desiredCell;
        this.isRedAlliance = isRedAlliance;
        this.maxSpeed = maxSpeed;

        tripPidController = new PIDController(4, 0, 0);
        turnPidController = new MMTurnPIDController();
        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        targetPose2d = MMField.getCellPose(desiredCell.get(), isRedAlliance.get());
        targetPosition = targetPose2d.getTranslation();
        turnPidController.initialize(targetPose2d.getRotation());
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("isRedAlliance: ", isRedAlliance.get());
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
    public boolean isFinished() {
        double tripLength = targetPosition.getDistance(rc.navigationSubsystem.getPose().getTranslation());
        boolean finishedTrip = tripLength < .05;
        SmartDashboard.putNumber("Trip Length Cell:", tripLength);
        SmartDashboard.putBoolean("Finished CellTranslate", finishedTrip);
        SmartDashboard.putString("Cell Position", targetPose2d.getTranslation().toString());
        return finishedTrip && turnPidController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
    }
}
