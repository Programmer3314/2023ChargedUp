// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class GripGrabRaw extends CommandBase {
    RobotContainer rc;

    public GripGrabRaw(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        rc.intakeSubsystem.setRetract();
        SmartDashboard.putString("Running gripGrab", "Yes");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
