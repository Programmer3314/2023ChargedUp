// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class LoadingPositionCmd extends CommandBase {
    private final RobotContainer rc;

    public LoadingPositionCmd(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.intakeSubsystem);
    }

    @Override
    public void initialize() {
        rc.intakeSubsystem.setLoading();

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return (Math.abs(rc.intakeSubsystem.getArmRotate() - Constants.Arm.Rotation.PositionControl.loading) < .01)
                && (Math.abs(rc.intakeSubsystem.getArmExtend() - Constants.Arm.Extend.PositionControl.loading) < .01);
    }

    @Override
    public void end(boolean interrupted) {
        rc.intakeSubsystem.stopArm();

    }
}
