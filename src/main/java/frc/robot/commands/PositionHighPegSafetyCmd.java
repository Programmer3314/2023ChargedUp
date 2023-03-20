// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PositionHighPegSafetyCmd extends CommandBase {
    RobotContainer rc;

    public PositionHighPegSafetyCmd(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.intakeSubsystem);
    }

    @Override
    public void initialize() {
        rc.intakeSubsystem.setHighPegSafety();

    }

    @Override
    public void execute() {

    }

    // TODO: This is HighPegSafety, but the isFinished is using .highPeg for the rotational constant
    // so as the arm rolls throught he highPeg position it stops. The are might continue to move (no stop)
    // but the release 
    @Override
    public boolean isFinished() {
        return Math.abs(rc.intakeSubsystem.getArmExtend() - Constants.Arm.Extend.PositionControl.highPeg) < .05
                && Math.abs(rc.intakeSubsystem.getArmRotate() - Constants.Arm.Rotation.PositionControl.highPeg) < .05;
    }

    @Override
    public void end(boolean interrupted) {
        // rc.intakeSubsystem.stopArm();
    }
}
