// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
public class TargetPegCmd extends CommandBase {

    double maxRotationSpeed;
    // MMTurnPIDController turnPidController;
    MMTurnPIDController turnPidController;
    private int cyclesOnTarget;
    Supplier<Boolean> cubeInIntake;
    private boolean isBackCam;
    private RobotContainer rc;

    public TargetPegCmd(RobotContainer rc, double maxRotationSpeed, Supplier<Boolean> cubeInIntake) {
        this.maxRotationSpeed = maxRotationSpeed;
        turnPidController = new MMTurnPIDController();
        this.cubeInIntake = cubeInIntake;
        this.rc = rc;

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        // navigationSubsystem.setFrontPipeline(1);

        turnPidController.initialize(new Rotation2d());
        cyclesOnTarget = 0;
        isBackCam = cubeInIntake.get();
    }

    @Override
    public void execute() {
        double rawTarget;
        if (isBackCam) {
            rawTarget = rc.navigationSubsystem.getBackTargetX();
            rc.navigationSubsystem.setBackPipeline(1);
        } else {
            rawTarget = rc.navigationSubsystem.getFrontTargetX();
            rc.navigationSubsystem.setFrontPipeline(1);
        }

        Rotation2d targetAngle = new Rotation2d(Math.toRadians(rawTarget));
        double correction = turnPidController.execute(targetAngle.getRadians());
        rc.swerveSubsystem.drive(0, 0, correction, true, rc.navigationSubsystem.getRotation2d());
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
            rc.navigationSubsystem.setFrontPipeline(0);
        }

    }

    @Override
    public boolean isFinished() {
        return cyclesOnTarget >= 75;
    }
}
