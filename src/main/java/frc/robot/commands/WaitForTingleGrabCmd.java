// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class WaitForTingleGrabCmd extends CommandBase{
    RobotContainer rc;

    public WaitForTingleGrabCmd(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.intakeSubsystem);
    }

    @Override
    public void initialize() {
      
    }

    @Override
    public void execute() {

    }

    @Override
    // wait for tingle, the set double solinoid to gripGrab
    public boolean isFinished() {
       return rc.intakeSubsystem.getSpideySense();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}


