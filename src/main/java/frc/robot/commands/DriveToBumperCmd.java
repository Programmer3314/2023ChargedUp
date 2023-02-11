// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utility.MMTurnPIDController;

/** Add your docs here. */
// store our rotation for gyro lock in init
// drive slowly forwards
// gyro lock angle
// stop when our speed is near 0
public class DriveToBumperCmd extends CommandBase {
    private double maxSpeed;
    // private PIDController tripPidController;
    private MMTurnPIDController turnPidController;
    private Rotation2d gyroLockAngle;
    private boolean pastStartUpFlag;
    private RobotContainer rc;
    private double tempMaxSpeed;

    public DriveToBumperCmd(RobotContainer rc,
            double maxSpeed) {
        this.rc = rc;
        this.maxSpeed = maxSpeed;
        // tripPidController = new PIDController(4, 0, 0);
        turnPidController = new MMTurnPIDController();

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {
        gyroLockAngle = rc.navigationSubsystem.getRotation2d();

        turnPidController.initialize(gyroLockAngle);
        pastStartUpFlag = false;
        if (rc.intakeSubsystem.getBeamBreak()) {
            tempMaxSpeed = -maxSpeed;
        }
        else{
            tempMaxSpeed=maxSpeed;
        }
    }

    @Override
    public void execute() {
        if (!pastStartUpFlag) {
            pastStartUpFlag = rc.swerveSubsystem.getAverageDriveVelocity() > .25;
        }

        SwerveModuleState[] desiredStates = new SwerveModuleState[] {
                new SwerveModuleState(tempMaxSpeed, new Rotation2d()),
                new SwerveModuleState(tempMaxSpeed, new Rotation2d()),
                new SwerveModuleState(tempMaxSpeed, new Rotation2d()),
                new SwerveModuleState(tempMaxSpeed, new Rotation2d())
        };
        rc.swerveSubsystem.setModuleStatesRaw(desiredStates, true);
        SmartDashboard.putNumber("Velocity", rc.swerveSubsystem.getAverageDriveVelocity());
        // double desiredTurn =
        // turnPidController.execute(navigationSubsystem.getHeadingRad());
        // double desiredX = Math.cos(gyroLockAngle.getRadians() * maxSpeed);
        // double desiredY = Math.sin(gyroLockAngle.getRadians() * maxSpeed);

        // swerveSubsystem.drive(desiredX, desiredY, desiredTurn, true,
        // navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        boolean droveToPeg = rc.swerveSubsystem.getAverageDriveVelocity() <= .22 && pastStartUpFlag;
        SmartDashboard.putBoolean("Drove To Peg", droveToPeg);
        return droveToPeg;
    }
}
