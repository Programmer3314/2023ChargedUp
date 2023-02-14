// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class SwerveJoystickCmd extends CommandBase {
    private final RobotContainer rc;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> overrideFieldOriented;

    public SwerveJoystickCmd(RobotContainer rc, Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.rc = rc;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.overrideFieldOriented = fieldOrientedFunction;

        addRequirements(rc.swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        rc.swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, !overrideFieldOriented.get(),
                rc.navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        rc.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
