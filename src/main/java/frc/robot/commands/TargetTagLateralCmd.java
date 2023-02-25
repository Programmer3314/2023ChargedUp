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
public class TargetTagLateralCmd extends CommandBase {

    double maxRotationSpeed;
    // MMTurnPIDController turnPidController;
    MMTurnPIDController turnPidController;
    PIDController targetPidController;
    private int cyclesOnTarget;
    private boolean isBackCam;
    private RobotContainer rc;
    private Supplier<Boolean> placeCube;
    private double rawTarget;

    public TargetTagLateralCmd(RobotContainer rc, double maxRotationSpeed,
            Supplier<Boolean> placeCube) {
        this.maxRotationSpeed = maxRotationSpeed;
        // turnPidController = new MMTurnPIDController();
        targetPidController = new PIDController(.06125, 0, 0);
        turnPidController = new MMTurnPIDController();
        this.rc = rc;
        this.placeCube = placeCube;

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        double angle = (placeCube.get() ^ rc.getIsRedAlliance()) ? 0 : Math.PI;

        cyclesOnTarget = 0;
        turnPidController.initialize(new Rotation2d(angle));
        if (placeCube.get()) {

            rc.navigationSubsystem.setBackPipeline(0);
        } else {

            rc.navigationSubsystem.setClawPipeline(0);
        }
    }

    @Override
    public void execute() {

        double correction = 0;
        

        double angleCorrection = turnPidController.execute(rc.navigationSubsystem.getRotation2d());
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

        rc.swerveSubsystem.drive(0, correction, angleCorrection, true, rc.navigationSubsystem.getRotation2d());
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
        return cyclesOnTarget >= 75;
    }
}
