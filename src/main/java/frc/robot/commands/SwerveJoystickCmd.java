// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class SwerveJoystickCmd extends CommandBase {

    private final MMSwerveSubsystem swerveSubsystem;
    private final MMNavigationSubsystem navigationSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> overrideFieldOriented;

    public SwerveJoystickCmd(MMSwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, MMNavigationSubsystem navigationSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.overrideFieldOriented = fieldOrientedFunction;
        this.navigationSubsystem = navigationSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

       swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed,!overrideFieldOriented.get() , navigationSubsystem.getRotation2d());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
