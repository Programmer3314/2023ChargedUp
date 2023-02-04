// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MMIntakeSubsystem;

/** Add your docs here. */
public class PickUpCubeCmd extends SequentialCommandGroup {
    public PickUpCubeCmd(MMIntakeSubsystem intakeSubsystem) {
        addCommands(
                new InstantCommand(intakeSubsystem::setIntakeFloor),
                new RunIntakeUntilBroken(intakeSubsystem),
                new InstantCommand(intakeSubsystem::setIntakeTravel)

        );
    }
}
