// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class RunIntakeUntilBrokenCmd extends CommandBase {
    RobotContainer rc;

    public RunIntakeUntilBrokenCmd(RobotContainer rc) {
        this.rc = rc;
    }

    @Override
    public void initialize() {
        rc.intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        rc.intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return rc.intakeSubsystem.getBeamBreak();
    }
}
