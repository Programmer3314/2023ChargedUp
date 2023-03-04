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
public class TargetPegDriveCmd extends CommandBase {
    double maxRotationSpeed;
    // MMTurnPIDController turnPidController;
    MMTurnPIDController turnPidController;
    PIDController targetPidController;
    private int cyclesOnTarget;
    Supplier<Boolean> cubeInIntake;
    private boolean isBackCam;
    private RobotContainer rc;
    private Supplier<Boolean> placeCube;
    private boolean pastStartUpFlag = false;
    private double driveSpeed;
    private double currentDriveSpeed;

    public TargetPegDriveCmd(RobotContainer rc, double maxRotationSpeed, Supplier<Boolean> placeCube,
            double driveSpeed) {
        this.maxRotationSpeed = maxRotationSpeed;
        // turnPidController = new MMTurnPIDController();
        targetPidController = new PIDController(.105, 0, 0);
        this.cubeInIntake = cubeInIntake;
        this.rc = rc;
        this.placeCube = placeCube;
        this.driveSpeed = driveSpeed;

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        currentDriveSpeed = driveSpeed;
        if (!rc.getIsRedAlliance()) {
            currentDriveSpeed = -driveSpeed;
        }
        // navigationSubsystem.setFrontPipeline(1);
        // targetPidController.i
        // turnPidController.initialize(new Rotation2d());
        cyclesOnTarget = 0;
        // isBackCam = cubeInIntake.get();
    }

    @Override
    public void execute() {
        if (!pastStartUpFlag) {
            pastStartUpFlag = rc.swerveSubsystem.getAverageDriveVelocity() > .15;
        }
        SmartDashboard.putBoolean("pastStartUp,", pastStartUpFlag);

        double rawTarget;
        double correction = 0;
        double angle = (placeCube.get() ^ rc.getIsRedAlliance()) ? 0 : Math.PI;
        if (placeCube.get()) {
            rawTarget = rc.navigationSubsystem.getBackTargetX();
            rc.navigationSubsystem.setBackPipeline(1);
        } else {
            rawTarget = rc.navigationSubsystem.getClawTargetX();
            rc.navigationSubsystem.setClawPipeline(1);
        }

        // Rotation2d targetAngle = new Rotation2d(Math.toRadians(rawTarget));
        // double correction = turnPidController.execute(targetAngle.getRadians());
        if (rc.navigationSubsystem.hasTargetClaw()) {
            correction = targetPidController.calculate(rc.getIsRedAlliance() ? rawTarget : -rawTarget);
        }
        SmartDashboard.putNumber("DrPeg Horizontal Correction", correction);
        SmartDashboard.putNumber("DrPeg Robot Angle", angle);
        rc.swerveSubsystem.drive(currentDriveSpeed, correction, 0, true, new Rotation2d(angle));
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
        SmartDashboard.putNumber("Drive Velocity: ", rc.swerveSubsystem.getAverageDriveVelocity());
        boolean droveToPeg = rc.swerveSubsystem.getAverageDriveVelocity() <= .8 && pastStartUpFlag;
        return cyclesOnTarget >= 75 && droveToPeg;
    }
}
