// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PositionLowPegCmd extends CommandBase {
    private final RobotContainer rc;

    public PositionLowPegCmd(RobotContainer rc) {
        this.rc = rc;
    }

    @Override
    public void initialize() {
        rc.intakeSubsystem.setLowPeg();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("FinishedLowPeg", Math.abs(rc.intakeSubsystem.getArmExtend() - Constants.Arm.Extend.PositionControl.lowPeg) < .04
        && Math.abs(rc.intakeSubsystem.getArmRotate() - Constants.Arm.Rotation.PositionControl.lowPeg) < .04);

    }

    @Override
    public boolean isFinished() {
        return Math.abs(rc.intakeSubsystem.getArmExtend() - Constants.Arm.Extend.PositionControl.lowPeg) < .04
                && Math.abs(rc.intakeSubsystem.getArmRotate() - Constants.Arm.Rotation.PositionControl.lowPeg) < .04;
    }

    @Override
    public void end(boolean interrupted) {
        rc.intakeSubsystem.stopArm();
    }
}
