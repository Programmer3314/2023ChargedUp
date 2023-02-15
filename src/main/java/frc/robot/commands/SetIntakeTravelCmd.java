// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// TODO: I think this needs to go away. Use IntakeHomeCmd instead.  


/** Add your docs here. */
public class SetIntakeTravelCmd extends SequentialCommandGroup{
    public  SetIntakeTravelCmd(RobotContainer rc){
     addCommands(
        new InstantCommand(rc.intakeSubsystem::setIntakeTravel)
        
        
     );   
    }
    

    
}

