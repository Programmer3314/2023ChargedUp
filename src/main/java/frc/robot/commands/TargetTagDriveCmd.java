// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class TargetTagDriveCmd extends CommandBase {
    double maxRotationSpeed;
    // MMTurnPIDController turnPidController;
    MMTurnPIDController turnPidController;
    PIDController targetPidController;
    private int cyclesOnTarget;
    private boolean isBackCam;
    private RobotContainer rc;
    private Supplier<Boolean> placeCube;
    private double rawTarget;
    private double driveSpeed;
    private boolean pastStartUpFlag = false;
    private double currentDriveSpeed;

    public TargetTagDriveCmd(RobotContainer rc, double maxRotationSpeed,
            Supplier<Boolean> placeCube, double driveSpeed) {
        this.driveSpeed = driveSpeed;
        this.maxRotationSpeed = maxRotationSpeed;
        // turnPidController = new MMTurnPIDController();
        targetPidController = new PIDController(.06125, 0, 0);
        this.rc = rc;
        this.placeCube = placeCube;

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        currentDriveSpeed = driveSpeed;
        if (!rc.getIsRedAlliance()) {
            currentDriveSpeed = -driveSpeed;
        }

        double angle = (placeCube.get() ^ rc.getIsRedAlliance()) ? 0 : Math.PI;

        cyclesOnTarget = 0;
        // turnPidController.initialize(new Rotation2d(angle));
        if (placeCube.get()) {

            rc.navigationSubsystem.setBackPipeline(0);
        } else {

            rc.navigationSubsystem.setClawPipeline(0);
        }
    }

    @Override
    public void execute() {
        if (!pastStartUpFlag) {
            pastStartUpFlag = rc.swerveSubsystem.getAverageDriveVelocity() > .25;
        }

        double correction = 0;

        double angleCorrection = (placeCube.get() ^ rc.getIsRedAlliance()) ? 0 : Math.PI;
        SmartDashboard.putNumber("Drive Velocity", rc.swerveSubsystem.getAverageDriveVelocity());
        SmartDashboard.putNumber("Current Angle Correction", angleCorrection);
        // Rotation2d targetAngle = new Rotation2d(Math.toRadians(rawTarget));
        // double correction = turnPidController.execute(targetAngle.getRadians());
        if (placeCube.get()) {
            if (rc.navigationSubsystem.hasTargetBack()) {
                rawTarget = rc.navigationSubsystem.getBackTargetX();
                correction = targetPidController.calculate(rawTarget);
            }
        } else {
            if (rc.navigationSubsystem.hasTargetClaw()) {
                rawTarget = rc.navigationSubsystem.getClawTargetX();
                correction = targetPidController.calculate(rawTarget);
            }
        }

        if (rc.navigationSubsystem.hasTargetClaw()) {
            correction = targetPidController.calculate(rc.getIsRedAlliance() ? rawTarget : -rawTarget);
        }
        SmartDashboard.putNumber("DrTag Horizontal Correction", correction);
        SmartDashboard.putNumber("DrTag Robot Angle", angleCorrection);

        rc.swerveSubsystem.drive(currentDriveSpeed, correction, 0, true, new Rotation2d(angleCorrection));
        if (Math.abs(rawTarget) < 3) {
            cyclesOnTarget++;
        } else {
            cyclesOnTarget = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
        if (isBackCam) {
            rc.navigationSubsystem.setBackPipeline(0);
        } else {
            rc.navigationSubsystem.setClawPipeline(0);
        }

    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("Drive Velocity: ",
                rc.swerveSubsystem.getAverageDriveVelocity());

        boolean droveToPeg = rc.swerveSubsystem.getAverageDriveVelocity() <= .8 && pastStartUpFlag;
        return cyclesOnTarget >= 75 && droveToPeg;
    }
}
