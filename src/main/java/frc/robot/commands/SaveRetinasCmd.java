// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class SaveRetinasCmd extends SequentialCommandGroup {
    public SaveRetinasCmd(RobotContainer rc) {
        addCommands(
                new InstantCommand(() -> rc.navigationSubsystem.setBackPipeline(0)),
                new InstantCommand(() -> rc.navigationSubsystem.setClawPipeline(0)));
        addRequirements(rc.intakeSubsystem);
    }
}
